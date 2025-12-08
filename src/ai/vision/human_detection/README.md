# Aerial Person Detection with YOLO

This project implements human/person detection in aerial/drone imagery using YOLO (You Only Look Once) deep learning model, specifically designed for the Small Object Aerial Person Dataset.

## 📁 Project Structure

```
vision/
├── data/
│   ├── raw/                    # Place your Small Object Aerial Person Dataset here
│   │   ├── images/
│   │   │   ├── train/
│   │   │   ├── val/
│   │   │   └── test/
│   │   └── labels/
│   │       ├── train/
│   │       ├── val/
│   │       └── test/
│   └── processed/              # Processed/augmented data
│
├── models/
│   ├── pretrained/             # Pre-trained YOLO weights
│   └── trained/                # Your trained models
│
├── configs/
│   ├── data_config.yaml        # Dataset configuration
│   ├── model_config.yaml       # Model training parameters
│   └── inference_config.yaml   # Inference settings
│
├── results/
│   ├── predictions/            # Detection results (images/videos)
│   └── metrics/                # Training metrics and plots
│
├── utils/
│   └── data_utils.py           # Helper functions
│
├── env/                        # Virtual environment
├── human_detection_yolo.ipynb  # Main notebook
├── data_installation.ipynb     # Data setup notebook
└── README.md
```

##  Getting Started

### 1. Dataset Preparation

Place your **Small Object Aerial Person Dataset** in the `data/raw/` directory with the following structure:

```
data/raw/
├── images/
│   ├── train/
│   ├── val/
│   └── test/
└── labels/
    ├── train/
    ├── val/
    └── test/
```

The labels should be in YOLO format (`.txt` files):
```
class_id x_center y_center width height
```
Run the data installation script to prepare your dataset:

```bash
python data_installation.py
```

This script handles:
- Downloading or validating the Small Object Aerial Person Dataset
- Organizing data into train/val/test splits
- Creating appropriate directory structures

### 2. Data Consolidation & Augmentation

Use the provided utilities to consolidate and augment your data:

```bash
# Consolidate data from multiple sources
python consolidate_data.py

# Combine all data
python compineAll.py

# Apply augmentations
python color_augmentation.py      # Add color variations
python noise_augmentation.py      # Add noise variations
python weather_augmentation.py    # Add weather effects
```

### 2. Configuration

Edit the configuration files in `configs/` according to your needs:

- **data_config.yaml**: Update paths and dataset parameters
- **model_config.yaml**: Adjust training hyperparameters
- **inference_config.yaml**: Set inference preferences

### 4. Training

Open `human_detection_yolo_ini.ipynb` and follow the notebook cells to:
1. Explore the dataset
2. Train the YOLO model
3. Evaluate performance
4. Run inference on test data

### 5. Model Selection

Choose from different YOLO models based on your needs:
- **YOLOv8n**: Fastest, smallest (for real-time applications)
- **YOLOv8s**: Small model with good balance
- **YOLOv8m**: Medium model (recommended for this task)
- **YOLOv8l**: Large model (better accuracy)
- **YOLOv8x**: Largest, most accurate (slower)

##  Features

- Complete pipeline for aerial person detection
- Configurable training parameters optimized for small objects
- Data visualization and analysis tools
- Model evaluation metrics and plots
- Easy inference on new images/videos
- Support for various YOLO model sizes

##  Key Parameters for Small Object Detection

The configuration is optimized for detecting small objects (persons in aerial imagery):

- Higher anchor scales for small objects
- Increased `max_det` for dense scenes
- Optimized augmentation for aerial views
- Fine-tuned confidence and IoU thresholds

##  Monitoring Training

Training progress is saved to `results/metrics/` including:
- Loss curves (box, class, DFL)
- Precision and Recall
- mAP@0.5 and mAP@0.5:0.95
- Confusion matrix
- Sample predictions

##  Expected Performance

For aerial person detection:
- **Small objects**: Persons typically 10-50 pixels
- **Dense crowds**: Multiple people in frame
- **Various heights**: Different drone altitudes
- **Challenge**: Occlusion, scale variation, lighting

##  Notes

- The project uses YOLOv8 from Ultralytics
- GPU recommended for training (CPU supported but slower)
- Adjust batch size based on available GPU memory
- Use data augmentation to improve robustness

##  Troubleshooting

- **Out of Memory**: Reduce batch size or image size
- **Poor Detection**: Adjust confidence threshold or train longer
- **Slow Training**: Use smaller model or reduce image size
- **Class Imbalance**: Adjust class weights in config

##  Resources

- [Ultralytics YOLOv8 Documentation](https://docs.ultralytics.com/)
- [YOLO Format Explanation](https://docs.ultralytics.com/datasets/detect/)
- Small Object Detection Best Practices

