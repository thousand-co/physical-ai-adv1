"""Supervised RGB decision tree trained on augmented simulation palette samples.

This is a scene-specific color classifier, not a pretrained object detector.
Training uses synthetic labeled RGB samples, never Gazebo model names at inference.
"""
from pathlib import Path
import json
import cv2
import numpy as np

PALETTE = {
    'blue': (0.122, 0.467, 0.706), 'orange': (1., .498, .055),
    'green': (.173, .627, .173), 'red': (.839, .153, .157),
    'purple': (.580, .404, .741), 'brown': (.549, .337, .294),
    'pink': (.890, .467, .761), 'gray': (.498, .498, .498),
    'olive': (.737, .741, .133), 'cyan': (.090, .745, .812),
}
LABELS = list(PALETTE) + ['background']


def training_samples(seed=42, count=500):
    rng = np.random.default_rng(seed)
    samples, labels = [], []
    for label, rgb in enumerate(PALETTE.values()):
        # Independent illumination and sensor noise; labels come from palette.
        values = np.array(rgb)[None, :] * rng.uniform(.65, 1.25, (count, 1))
        values += rng.normal(0, .012, (count, 3))
        samples.append(np.clip(values, 0, 1))
        labels.extend([label] * count)
    # Background colors are specific to this scene; gray remains ambiguous.
    for rgb in ((.7,.7,.7), (.25,.25,.28), (.9,.8,.25), (.1,.9,.3), (0,0,0), (1,1,1)):
        values = np.array(rgb)[None, :] + rng.normal(0, .008, (count, 3))
        samples.append(np.clip(values, 0, 1))
        labels.extend([len(PALETTE)] * count)
    return np.vstack(samples).astype(np.float32), np.array(labels, np.int32)


def train_model(path):
    x, y = training_samples()
    tree = cv2.ml.DTrees_create()
    tree.setMaxDepth(16)
    tree.setMinSampleCount(5)
    tree.setCVFolds(0)
    tree.train(x, cv2.ml.ROW_SAMPLE, y)
    tree.save(str(path))
    test_x, test_y = training_samples(seed=123, count=200)
    predicted = tree.predict(test_x)[1].ravel().astype(int)
    report = {'algorithm': 'OpenCV DTrees supervised RGB classifier',
              'training_seed': 42, 'evaluation_seed': 123,
              'training_samples': len(y), 'evaluation_samples': len(test_y),
              'synthetic_accuracy': float(np.mean(predicted == test_y)),
              'per_class_accuracy': {name: float(np.mean(predicted[test_y == i] == i))
                                     for i, name in enumerate(LABELS)},
              'limitation': 'Synthetic palette validation only; not real-world object recognition.'}
    Path(path).with_suffix('.json').write_text(json.dumps(report, indent=2))
    return report


class LearnedColorClassifier:
    def __init__(self, model_path):
        self.model = cv2.ml.DTrees_load(str(model_path))
        if not self.model.isTrained():
            raise ValueError('Color classifier is not trained')

    def mask(self, image_bgr, color):
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        _, predictions = self.model.predict(rgb.reshape(-1, 3).astype(np.float32) / 255.)
        return (predictions.reshape(rgb.shape[:2]) == LABELS.index(color)).astype(np.uint8) * 255
