import cv2
import numpy as np
import random
from pathlib import Path
import os
import shutil
from tqdm import tqdm


class DroneWeatherAugmentor:
    def __init__(self, seed=42):
        random.seed(seed)
        np.random.seed(seed)

    def add_fog(self, image):
        # Simulate atmospheric fog or haze
        h, w = image.shape[:2]
        fog_intensity = random.uniform(0.3, 0.7)
        fog = np.ones_like(image, dtype=np.float32) * 255
        blended = cv2.addWeighted(image.astype(np.float32), 1 - fog_intensity, fog, fog_intensity, 0)
        return blended.astype(np.uint8)

    def add_rain(self, image):
        # Simulate rain streaks using random lines
        rain_density = random.randint(300, 700)
        rain_intensity = random.uniform(0.3, 0.7)
        rain = np.zeros_like(image, dtype=np.uint8)

        for _ in range(rain_density):
            x1 = random.randint(0, image.shape[1])
            y1 = random.randint(0, image.shape[0])
            length = random.randint(10, 20)
            thickness = random.randint(1, 2)
            angle = random.uniform(-30, 30)
            x2 = int(x1 + length * np.sin(np.deg2rad(angle)))
            y2 = int(y1 + length * np.cos(np.deg2rad(angle)))
            cv2.line(rain, (x1, y1), (x2, y2), (255, 255, 255), thickness)

        rain = cv2.blur(rain, (3, 3))
        combined = cv2.addWeighted(image, 1, rain, rain_intensity, 0)
        return combined

    def add_snow(self, image):
        # Simulate snowflakes as white blurred dots
        snow_density = random.randint(200, 400)
        snow = image.copy()
        for _ in range(snow_density):
            x = random.randint(0, image.shape[1] - 1)
            y = random.randint(0, image.shape[0] - 1)
            radius = random.randint(1, 3)
            cv2.circle(snow, (x, y), radius, (255, 255, 255), -1)

        snow = cv2.GaussianBlur(snow, (3, 3), 0)
        snow = cv2.addWeighted(image, 0.8, snow, 0.4, 0)
        return snow

    def add_dust(self, image):
        # Simulate dust or sandstorm haze
        h, w = image.shape[:2]
        overlay = np.full((h, w, 3), (194, 178, 128), dtype=np.uint8)  # Sand color
        alpha = random.uniform(0.2, 0.5)
        dusty = cv2.addWeighted(image, 1 - alpha, overlay, alpha, 0)
        return dusty

    def add_storm(self, image):
        # Simulate dark cloudy storm atmosphere
        gray_overlay = np.zeros_like(image, dtype=np.uint8)
        gray_overlay[:] = (40, 40, 40)
        alpha = random.uniform(0.4, 0.7)
        stormy = cv2.addWeighted(image, 1 - alpha, gray_overlay, alpha, 0)
        stormy = cv2.GaussianBlur(stormy, (5, 5), 1)
        return stormy

    def add_flames_glow(self, image):
        # Simulate fire glow or sunset reflection (reddish)
        h, w = image.shape[:2]
        overlay = np.full((h, w, 3), (0, 80, 200), dtype=np.uint8)  # Warm orange-red tint
        intensity = random.uniform(0.2, 0.5)
        glow = cv2.addWeighted(image, 1 - intensity, overlay, intensity, 0)
        return glow

    def apply_weather_aug(self, image, aug_type):
        if aug_type == "fog":
            return self.add_fog(image)
        elif aug_type == "rain":
            return self.add_rain(image)
        elif aug_type == "snow":
            return self.add_snow(image)
        elif aug_type == "dust":
            return self.add_dust(image)
        elif aug_type == "storm":
            return self.add_storm(image)
        elif aug_type == "flames_glow":
            return self.add_flames_glow(image)
        else:
            return image


if __name__ == "__main__":
    base_dir = Path("/home/abdallah/Coding/drones/data/raw")
    output_base = Path("/home/abdallah/Coding/drones/data/weather_augmentation")
    augmenter = DroneWeatherAugmentor(seed=42)

    weather_types = [
        "fog",
        "rain",
        "snow",
        "dust",
        "storm",
        "flames_glow",
    ]

    splits = ["train", "val", "test"]

    for weather_type in tqdm(weather_types, desc=" Applying Weather Augmentations", position=0, leave=True):
        for split in splits:
            input_dir = base_dir / "images" / split
            label_dir = base_dir / "labels" / split

            img_files = [f for f in os.listdir(input_dir) if f.lower().endswith((".jpg", ".jpeg", ".png"))]

            output_img_dir = output_base / weather_type / "images" / split
            output_lbl_dir = output_base / weather_type / "labels" / split
            output_img_dir.mkdir(parents=True, exist_ok=True)
            output_lbl_dir.mkdir(parents=True, exist_ok=True)

            img_progress_bar = tqdm(img_files, desc=f"   {weather_type} - {split} split", position=1, leave=False)

            for img_file in img_progress_bar:
                img_path = input_dir / img_file
                label_path = label_dir / img_file.replace(".jpg", ".txt").replace(".png", ".txt")

                image = cv2.imread(str(img_path))
                if image is None:
                    tqdm.write(f"Skipped unreadable image: {img_file}")
                    continue

                augmented = augmenter.apply_weather_aug(image, weather_type)

                save_img_path = output_img_dir / img_file
                cv2.imwrite(str(save_img_path), augmented)

                if label_path.exists():
                    save_label_path = output_lbl_dir / label_path.name
                    shutil.copy(label_path, save_label_path)

    print("\nweather augmentations completed successfully!")
    print(f"Results saved in: {output_base.resolve()}")
