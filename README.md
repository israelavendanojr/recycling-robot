# recycling-robot
Raspberry Pi robot to automate recycling sorting


# Raspberry Pi Inference

## Quick Commands

```bash
# Test model (no camera)
python scripts/inference_test.py --test_model

# Single photo classification  
python scripts/inference_test.py --single_shot

# Continuous monitoring (default)
python scripts/inference_test.py
```

## Options

- `--model_path models/recycler.pth` - specify model file
- `--single_shot` - classify once and exit
- `--delay 3.0` - seconds between captures (continuous mode)
- `--save_frames` - save photos for debugging
- `--test_model` - test model loading without camera

## Classes

Detects: `cardboard`, `glass`, `metal`, `plastic`, `trash`

## Stop continuous mode

Press `Ctrl+C`