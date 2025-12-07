import os
import shutil
import random
BASE_DIR = '/home/abdallah/Coding/drones/data' 

AUGMENTATION_CATEGORIES = ['color_augmentation', 'noise_augmentation', 'weather_augmentation']

SUB_AUGMENTATIONS = [
    'brightness', 'contrast', 'saturation', 'shadow', 'sunlight',
    'compression', 'gaussian_blur', 'gaussian_noise', 'motion_blur',
    'dust', 'flames_glow', 'fog', 'rain', 'snow', 'storm'
]

OUTPUT_TRAIN_IMG = os.path.join(BASE_DIR, 'final_train/Images/train')
OUTPUT_TRAIN_LBL = os.path.join(BASE_DIR, 'final_train/Labels/train')

# Number of random images to SELECT from each augmentation type
SAMPLES_PER_AUGMENTATION_TYPE = 300 


def setup_directories():
    if os.path.exists(os.path.join(BASE_DIR, 'final_train')):
        shutil.rmtree(os.path.join(BASE_DIR, 'final_train'))
        print("Cleaned existing 'final_train' directory.")
        
    os.makedirs(OUTPUT_TRAIN_IMG, exist_ok=True)
    os.makedirs(OUTPUT_TRAIN_LBL, exist_ok=True)
    print("Created new 'final_train' directories.")


def copy_raw_data():
    raw_img_path = os.path.join(BASE_DIR, 'raw/images/train')
    raw_lbl_path = os.path.join(BASE_DIR, 'raw/labels/train')
    
    # Check if raw data path exists based on your provided structure
    if not os.path.exists(raw_img_path):
        print(f"ERROR: Raw data path not found. Please ensure 'data/raw/images/train' exists.")
        return
        
    print(f"Copying RAW data from {raw_img_path}...")
    
    img_count = 0
    # Copy Images
    for filename in os.listdir(raw_img_path):
        if filename.endswith(('.jpg', '.png')):
            shutil.copy(os.path.join(raw_img_path, filename), OUTPUT_TRAIN_IMG)
            img_count += 1
            
    # Copy Labels
    for filename in os.listdir(raw_lbl_path):
        if filename.endswith('.txt'):
            shutil.copy(os.path.join(raw_lbl_path, filename), OUTPUT_TRAIN_LBL)
            
    print(f"Copied {img_count} RAW images/labels.")


def copy_random_augmented_data():
    total_augmented_copied = 0
    
    for aug_name in SUB_AUGMENTATIONS:
        source_img_dir = None
        source_lbl_dir = None
        for cat in AUGMENTATION_CATEGORIES:
            potential_path = os.path.join(BASE_DIR, cat, aug_name, 'images', 'train')
            if os.path.exists(potential_path):
                source_img_dir = potential_path
                source_lbl_dir = os.path.join(BASE_DIR, cat, aug_name, 'labels', 'train') 
                break
        
        if source_img_dir and os.path.exists(source_img_dir):
            all_images = [f for f in os.listdir(source_img_dir) if f.endswith(('.jpg', '.png'))]
            
            # Select random files
            num_samples = min(SAMPLES_PER_AUGMENTATION_TYPE, len(all_images))
            selected_files = random.sample(all_images, num_samples)
            
            # Copy selected files and their corresponding labels with renaming
            for img_file in selected_files:
                base_name, ext = os.path.splitext(img_file)
                label_file = base_name + '.txt'
                
                # Create the new file names by appending the augmentation type
                new_base_name = f"{base_name}_{aug_name}"
                new_img_file = new_base_name + ext
                new_label_file = new_base_name + '.txt'
                
                # --- Copy and Rename Image ---
                src_img_path = os.path.join(source_img_dir, img_file)
                dst_img_path = os.path.join(OUTPUT_TRAIN_IMG, new_img_file)
                shutil.copy(src_img_path, dst_img_path)
                
                # --- Copy and Rename Label ---
                src_lbl_path = os.path.join(source_lbl_dir, label_file)
                dst_lbl_path = os.path.join(OUTPUT_TRAIN_LBL, new_label_file)
                if os.path.exists(src_lbl_path):
                    shutil.copy(src_lbl_path, dst_lbl_path)
                    
                total_augmented_copied += 1
                
            print(f"Copied {num_samples} samples from '{aug_name}'. Total: {total_augmented_copied}")
        else:
            print(f"Skipping '{aug_name}': Source directory not found at expected path.")


if __name__ == "__main__":
    setup_directories()
    copy_raw_data()
    copy_random_augmented_data()
    
    final_img_count = len(os.listdir(OUTPUT_TRAIN_IMG))
    print(f"\n--- DONE ---")
    print(f"Total training images in 'final_train': {final_img_count}")