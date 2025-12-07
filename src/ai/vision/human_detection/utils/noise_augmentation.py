import cv2
import numpy as np
import random
from pathlib import Path
import os
import shutil
from tqdm import tqdm


class DroneNoiseAugmentor:
    def __init__(self, seed=42):
        random.seed(seed)
        np.random.seed(seed)

    # --- Noise Methods
    def add_gaussian_noise(self, image):
            sigma = random.uniform(0.005, 0.05) 
            noisy = image.astype(np.float32) / 255.0
            noise = np.random.normal(0, sigma, image.shape)
            noisy = noisy + noise
            noisy = np.clip(noisy, 0, 1)
            return (noisy * 255).astype(np.uint8)

    def add_speckle_noise(self, image):
            sigma = random.uniform(0.03, 0.08)
            gauss = np.random.randn(*image.shape) * sigma
            noisy = image + image * gauss
            noisy = np.clip(noisy, 0, 255)
            return noisy.astype(np.uint8)

    def add_compression(self, image):
            # Randomly choose a JPEG quality level between 50 (heavy) and 90 (light)
            quality = random.randint(50, 90)
            
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            _, encimg = cv2.imencode('.jpg', image, encode_param)
            # Decode the JPEG bytes back to an image
            decimg = cv2.imdecode(encimg, 1)
            return decimg
        
    def add_motion_blur(self, image):
            kernel_size = random.choice([3, 5, 7, 9])
            angle = random.uniform(0, 180)          
            kernel = np.zeros((kernel_size, kernel_size))
            kernel[int((kernel_size - 1) / 2), :] = np.ones(kernel_size)
            kernel /= kernel_size
            center = ((kernel_size - 1) / 2, (kernel_size - 1) / 2)
            M = cv2.getRotationMatrix2D(center, angle, 1.0) 
            kernel_rotated = cv2.warpAffine(kernel, M, (kernel_size, kernel_size))
            kernel_rotated = kernel_rotated / np.sum(kernel_rotated)
            return cv2.filter2D(image, -1, kernel_rotated)

    def add_gaussian_blur(self, image):
        ksize = random.choice([3, 5, 7, 9])
        sigmaX = random.uniform(0.5, 3.0) 
        return cv2.GaussianBlur(image, (ksize, ksize), sigmaX)

    def add_salt_pepper_noise(self, image):
            amount = random.uniform(0.002, 0.015) 
            
            noisy = np.copy(image)
            
            # Salt
            num_salt = np.ceil(amount * image.size * 0.5)
            coords = [np.random.randint(0, i - 1, int(num_salt)) for i in image.shape]
            noisy[tuple(coords)] = 255

            # Pepper
            num_pepper = np.ceil(amount * image.size * 0.5)
            coords = [np.random.randint(0, i - 1, int(num_pepper)) for i in image.shape]
            noisy[tuple(coords)] = 0
            
            return noisy

    def apply_noise(self, image, noise_type):
        if noise_type == "gaussian_noise":
            return self.add_gaussian_noise(image)
        elif noise_type == "speckle_noise":
            return self.add_speckle_noise(image)
        elif noise_type == "compression":
            return self.add_compression(image)
        elif noise_type == "motion_blur":
            return self.add_motion_blur(image)
        elif noise_type == "gaussian_blur":
            return self.add_gaussian_blur(image)
        elif noise_type == "salt_pepper":
            return self.add_salt_pepper_noise(image)
        else:
            return image

if __name__ == "__main__":
    base_dir = Path("/home/abdallah/Coding/drones/data/raw")
    output_base = Path("/home/abdallah/Coding/drones/data/noise_augmentation")
    augmenter = DroneNoiseAugmentor(seed=42)
    noise_types = [
        "gaussian_noise",
        "speckle_noise",
        "compression",
        "motion_blur",
        "gaussian_blur",
        "salt_pepper",
    ]

    splits = ["train", "val", "test"]
    for noise_type in tqdm(noise_types, desc="⚙️ Augmenting Noise Types", position=0, leave=True):
        for split in splits:
            input_dir = base_dir / "images" / split
            label_dir = base_dir / "labels" / split

            img_files = [f for f in os.listdir(input_dir) if f.lower().endswith((".jpg", ".jpeg", ".png"))]
            output_img_dir = output_base / noise_type / "images" / split
            output_lbl_dir = output_base / noise_type / "labels" / split
            output_img_dir.mkdir(parents=True, exist_ok=True)
            output_lbl_dir.mkdir(parents=True, exist_ok=True)
            img_progress_bar = tqdm(img_files, desc=f"   {noise_type[:1].upper()}{noise_type[1:]} - {split} split", position=1, leave=False)

            for img_file in img_progress_bar:
                img_path = input_dir / img_file
                label_path = label_dir / img_file.replace(".jpg", ".txt").replace(".png", ".txt")

                image = cv2.imread(str(img_path))
                if image is None:
                    tqdm.write(f"Skipped unreadable image: {img_file}")
                    continue
                augmented = augmenter.apply_noise(image, noise_type)
                save_img_path = output_img_dir / img_file
                cv2.imwrite(str(save_img_path), augmented)

                if label_path.exists():
                    save_label_path = output_lbl_dir / label_path.name
                    shutil.copy(label_path, save_label_path)

    print("\nnoise augmentations completed successfully!")
    print(f"Results saved in: {output_base.resolve()}")