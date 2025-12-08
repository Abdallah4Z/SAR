import os
import shutil

base_dir = "/home/abdallah/Coding/drones/data"

combined_dir = os.path.join(base_dir, "combined")
images_combined = os.path.join(combined_dir, "images")
labels_combined = os.path.join(combined_dir, "labels")
for path in [images_combined, labels_combined]:
    os.makedirs(os.path.join(path, "train"), exist_ok=True)
    os.makedirs(os.path.join(path, "val"), exist_ok=True)
    os.makedirs(os.path.join(path, "test"), exist_ok=True)

augmentation_folders = [
    "color_augmentation",
    "noise_augmentation",
    "weather_augmentation",
]

image_exts = [".jpg"]

def copy_with_new_name(src_img, src_label, dst_img_dir, dst_label_dir, technique):
    filename, ext = os.path.splitext(os.path.basename(src_img))
    new_name = f"{filename}_{technique}{ext}"
    new_label_name = f"{filename}_{technique}.txt"

    shutil.copy2(src_img, os.path.join(dst_img_dir, new_name))
    if os.path.exists(src_label):
        shutil.copy2(src_label, os.path.join(dst_label_dir, new_label_name))

# Traverse augmentation folders
for aug_folder in augmentation_folders:
    aug_path = os.path.join(base_dir, aug_folder)

    for technique in os.listdir(aug_path):
        technique_path = os.path.join(aug_path, technique)
        if not os.path.isdir(technique_path):
            continue

        for split in ["train", "val", "test"]:
            img_src_dir = os.path.join(technique_path, "images", split)
            lbl_src_dir = os.path.join(technique_path, "labels", split)

            img_dst_dir = os.path.join(images_combined, split)
            lbl_dst_dir = os.path.join(labels_combined, split)

            if not os.path.exists(img_src_dir):
                continue

            for img_name in os.listdir(img_src_dir):
                if os.path.splitext(img_name)[1].lower() not in image_exts:
                    continue

                img_path = os.path.join(img_src_dir, img_name)
                lbl_name = os.path.splitext(img_name)[0] + ".txt"
                lbl_path = os.path.join(lbl_src_dir, lbl_name)

                copy_with_new_name(img_path, lbl_path, img_dst_dir, lbl_dst_dir, technique)

print("combined successfully!")
print(f"Images stored in: {images_combined}")
print(f"Labels stored in: {labels_combined}")
