import cv2
import numpy as np
import random
from pathlib import Path
import os
import shutil
from tqdm import tqdm


class DroneColorAugmentor:
    def __init__(self, seed=42):
        random.seed(seed)
        np.random.seed(seed)

    def adjust_brightness(self, image):
        # Simulate different lighting or time of day
        factor = random.uniform(0.5, 1.5)  # 0.5 = darker, 1.5 = brighter
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv = np.array(hsv, dtype=np.float32)
        hsv[:, :, 2] = hsv[:, :, 2] * factor
        hsv[:, :, 2] = np.clip(hsv[:, :, 2], 0, 255)
        return cv2.cvtColor(np.uint8(hsv), cv2.COLOR_HSV2BGR)

    def adjust_contrast(self, image):
        # Simulate camera exposure differences
        alpha = random.uniform(0.8, 1.5)  # contrast control
        beta = random.uniform(-30, 30)    # brightness bias
        adjusted = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        return adjusted

    def adjust_saturation(self, image):
        # Simulate dull or vibrant lighting
        factor = random.uniform(0.6, 1.5)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[:, :, 1] *= factor
        hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
        return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    
    def add_shadow(self, image):
        h, w = image.shape[:2]
        
        # Define random diagonal strip for the shadow area
        top_x, bottom_x = np.random.randint(0, w, 2)
        top_y, bottom_y = 0, h
        strip_width = random.randint(50, 150)
        polygon = np.array([[top_x, top_y], 
                            [bottom_x, bottom_y], 
                            [bottom_x + strip_width, bottom_y], 
                            [top_x + strip_width, top_y]], dtype=np.int32)
        
        # Create a shadow mask
        shadow_mask = np.zeros_like(image, dtype=np.uint8)
        cv2.fillPoly(shadow_mask, [polygon], (1, 1, 1)) 
        darken_factor = random.uniform(0.3, 0.7) # Darkens pixels, 0.3=very dark
        alpha = random.uniform(0.3, 0.6)        
        darkened_image = (image * darken_factor).astype(np.uint8)
        shadowed = image.copy()
        shadowed[shadow_mask[:,:,0] == 1] = (image[shadow_mask[:,:,0] == 1] * darken_factor).astype(np.uint8)
        
        return shadowed

    def add_sunlight(self, image):
        """Simulate sun glare/bloom with a random, warm-colored circle."""
        overlay = image.copy()
        output = image.copy()

        h, w = image.shape[:2]
        center_x = random.choice([0, w]) 
        center_y = random.choice([0, h])
        radius = random.randint(200, 400) 
        intensity = random.uniform(0.15, 0.5) 
        glare_color = (random.randint(200, 255), random.randint(200, 255), random.randint(180, 255))
        cv2.circle(overlay, (center_x, center_y), radius, glare_color, -1)
        cv2.addWeighted(overlay, intensity, output, 1 - intensity, 0, output)

        return output

    def apply_color_aug(self, image, aug_type):
        if aug_type == "brightness":
            return self.adjust_brightness(image)
        elif aug_type == "contrast":
            return self.adjust_contrast(image)
        elif aug_type == "saturation":
            return self.adjust_saturation(image)
        elif aug_type == "shadow":
            return self.add_shadow(image)
        elif aug_type == "sunlight":
            return self.add_sunlight(image)
        else:
            return image



if __name__ == "__main__":
    base_dir = Path("/home/abdallah/Coding/drones/data/raw")
    output_base = Path("/home/abdallah/Coding/drones/data/color_augmentation")
    augmenter = DroneColorAugmentor(seed=42)

    color_types = [
        "brightness",
        "contrast",
        "saturation",
        "shadow",
        "sunlight",
    ]

    splits = ["train", "val", "test"]

    for color_type in tqdm(color_types, desc="Applying Color Augmentations", position=0, leave=True):
        for split in splits:
            input_dir = base_dir / "images" / split
            label_dir = base_dir / "labels" / split

            img_files = [f for f in os.listdir(input_dir) if f.lower().endswith((".jpg"))]

            output_img_dir = output_base / color_type / "images" / split
            output_lbl_dir = output_base / color_type / "labels" / split
            output_img_dir.mkdir(parents=True, exist_ok=True)
            output_lbl_dir.mkdir(parents=True, exist_ok=True)

            img_progress_bar = tqdm(img_files, desc=f"   {color_type} - {split} split", position=1, leave=False)

            for img_file in img_progress_bar:
                img_path = input_dir / img_file
                label_path = label_dir / img_file.replace(".jpg", ".txt").replace(".png", ".txt")

                image = cv2.imread(str(img_path))
                if image is None:
                    tqdm.write(f"Skipped unreadable image: {img_file}")
                    continue

                augmented = augmenter.apply_color_aug(image, color_type)

                save_img_path = output_img_dir / img_file
                cv2.imwrite(str(save_img_path), augmented)

                if label_path.exists():
                    save_label_path = output_lbl_dir / label_path.name
                    shutil.copy(label_path, save_label_path)

    print("\ncolor augmentations completed successfully!")
    print(f"Results saved in: {output_base.resolve()}")
