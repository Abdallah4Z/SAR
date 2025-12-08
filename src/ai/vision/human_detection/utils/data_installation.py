import os
import zipfile
import wget
from ultralytics import YOLO

data_dir = 'Small_Object_Aerial_Person_Dataset'
os.makedirs(data_dir, exist_ok=True)

# 1. Download Images (approx 3.7 GB)
images_url = 'https://zenodo.org/records/7740081/files/Images.zip?download=1'
print("Downloading Images...")
wget.download(images_url, out=data_dir)

# 2. Download Annotations (approx 4.1 MB)
annotations_url = 'https://zenodo.org/records/7740081/files/Annotations.zip?download=1'
print("\nDownloading Annotations...")
wget.download(annotations_url, out=data_dir)

# 3. Extract the Zipped Files
print("\nExtracting files...")
with zipfile.ZipFile(os.path.join(data_dir, 'Images.zip'), 'r') as zip_ref:
    zip_ref.extractall(data_dir)
with zipfile.ZipFile(os.path.join(data_dir, 'Annotations.zip'), 'r') as zip_ref:
    zip_ref.extractall(data_dir)
print("Extraction complete.")