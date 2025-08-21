# Models Directory

This directory contains machine learning models for the recycling robot classifier.

## Current Models

- `recycler.pth` - PyTorch model for recycling classification (placeholder)

## Adding Custom Models

1. **PyTorch Models**: Place `.pth` or `.pt` files here
2. **ONNX Models**: Convert to PyTorch format or modify classifier node
3. **TensorFlow Models**: Convert to PyTorch format

## Model Format

The classifier expects models that:
- Accept 224x224 RGB images
- Output class probabilities for recycling categories
- Are compatible with PyTorch JIT

## Default Classes

The classifier supports these recycling categories:
- cardboard
- glass  
- metal
- plastic
- trash

## Training Your Own Model

1. Collect labeled images of recyclable materials
2. Train a PyTorch model (ResNet, EfficientNet, etc.)
3. Export as JIT model: `torch.jit.script(model)`
4. Save as `.pth` file in this directory
5. Update `config/camera.yaml` with your model path
