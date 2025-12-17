"""
Transformer-Based Textual Entailment Recognition (Phase 3)

This module implements a state-of-the-art textual entailment recognizer using
pre-trained transformer models fine-tuned on NLI datasets (SNLI, MultiNLI).

Models used:
- cross-encoder/nli-deberta-v3-small: Compact, accurate NLI model
- Alternative: facebook/bart-large-mnli (larger, slightly better)

Expected accuracy: 85-90% on standard benchmarks

Author: mostafa1712002
License: MIT
"""

import numpy as np
from collections import Counter

try:
    from transformers import pipeline, AutoModelForSequenceClassification, AutoTokenizer
    TRANSFORMERS_AVAILABLE = True
except ImportError:
    TRANSFORMERS_AVAILABLE = False
    print("Warning: transformers not installed.")
    print("Install with: pip install transformers")


class TransformerEntailmentRecognizer:
    """
    State-of-the-art textual entailment recognizer using pre-trained NLI models.

    This class uses transformer models that have been fine-tuned on large-scale
    NLI datasets (SNLI, MultiNLI) containing millions of human-annotated examples.

    Advantages over embedding-based approach:
    - Cross-attention between premise and hypothesis
    - Trained specifically for NLI task
    - Better handling of complex linguistic phenomena
    - State-of-the-art accuracy (~88% on SNLI test set)

    Models available:
    - 'cross-encoder/nli-deberta-v3-small': Fast, accurate (default)
    - 'cross-encoder/nli-deberta-v3-base': Better accuracy, slower
    - 'facebook/bart-large-mnli': Alternative architecture
    """

    def __init__(self, model_name='cross-encoder/nli-deberta-v3-small'):
        """
        Initialize the transformer-based recognizer.

        Args:
            model_name (str): HuggingFace model name for NLI
        """
        if not TRANSFORMERS_AVAILABLE:
            raise ImportError(
                "transformers is required. "
                "Install with: pip install transformers"
            )

        self.model_name = model_name
        print(f"Loading transformer model: {model_name}...")

        # Use the zero-shot classification pipeline for NLI
        # This handles the model loading and inference efficiently
        self.classifier = pipeline(
            "zero-shot-classification",
            model=model_name,
            device=-1  # CPU, use 0 for GPU
        )

        # Label mapping
        self.labels = ['entailment', 'contradiction', 'neutral']

        print("Transformer model loaded successfully!")

    def recognize(self, premise, hypothesis):
        """
        Recognize textual entailment using the transformer model.

        The model processes the premise-hypothesis pair through cross-attention
        layers to determine their logical relationship.

        Args:
            premise (str): The premise text
            hypothesis (str): The hypothesis text

        Returns:
            str: 'ENTAILMENT', 'CONTRADICTION', or 'NEUTRAL'
        """
        # Format input for the model
        # The model expects: premise [SEP] hypothesis
        result = self.classifier(
            premise,
            candidate_labels=self.labels,
            hypothesis_template=hypothesis + " {}"
        )

        # Get the top prediction
        top_label = result['labels'][0]

        return top_label.upper()

    def recognize_with_confidence(self, premise, hypothesis):
        """
        Recognize textual entailment with confidence score.

        Args:
            premise (str): The premise text
            hypothesis (str): The hypothesis text

        Returns:
            tuple: (label, confidence_score)
        """
        result = self.classifier(
            premise,
            candidate_labels=self.labels,
            hypothesis_template=hypothesis + " {}"
        )

        top_label = result['labels'][0].upper()
        top_score = result['scores'][0]

        return top_label, round(top_score, 3)

    def recognize_batch(self, pairs):
        """
        Batch processing for multiple premise-hypothesis pairs.

        Args:
            pairs (list): List of {'premise': str, 'hypothesis': str} dicts

        Returns:
            list: List of predicted labels
        """
        results = []
        for pair in pairs:
            label = self.recognize(pair['premise'], pair['hypothesis'])
            results.append(label)
        return results

    def recognize_batch_with_confidence(self, pairs):
        """
        Batch processing with confidence scores.

        Args:
            pairs (list): List of {'premise': str, 'hypothesis': str} dicts

        Returns:
            list: List of (label, confidence) tuples
        """
        results = []
        for pair in pairs:
            label, confidence = self.recognize_with_confidence(
                pair['premise'], pair['hypothesis']
            )
            results.append((label, confidence))
        return results

    def evaluate(self, pairs, gold_labels):
        """
        Evaluate the recognizer against gold standard labels.

        Args:
            pairs (list): List of premise-hypothesis pairs
            gold_labels (list): List of true labels

        Returns:
            dict: Evaluation metrics
        """
        predictions = self.recognize_batch(pairs)

        # Calculate accuracy
        correct = sum(1 for pred, gold in zip(predictions, gold_labels) if pred == gold)
        accuracy = correct / len(gold_labels) if gold_labels else 0.0

        # Calculate per-class metrics
        labels = ['ENTAILMENT', 'CONTRADICTION', 'NEUTRAL']
        per_class = {}

        for label in labels:
            tp = sum(1 for pred, gold in zip(predictions, gold_labels)
                     if pred == label and gold == label)
            fp = sum(1 for pred, gold in zip(predictions, gold_labels)
                     if pred == label and gold != label)
            fn = sum(1 for pred, gold in zip(predictions, gold_labels)
                     if pred != label and gold == label)

            precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
            recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
            f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

            per_class[label] = {
                'precision': round(precision, 4),
                'recall': round(recall, 4),
                'f1': round(f1, 4),
                'support': sum(1 for g in gold_labels if g == label)
            }

        # Macro F1
        macro_f1 = sum(per_class[label]['f1'] for label in labels) / len(labels)

        # Confusion matrix
        confusion_matrix = {gold: Counter() for gold in labels}
        for pred, gold in zip(predictions, gold_labels):
            confusion_matrix[gold][pred] += 1

        return {
            'accuracy': round(accuracy, 4),
            'per_class': per_class,
            'macro_f1': round(macro_f1, 4),
            'predictions': predictions,
            'confusion_matrix': dict(confusion_matrix),
            'total_samples': len(gold_labels)
        }

    def print_evaluation_report(self, metrics):
        """Print a formatted evaluation report."""
        print("\n" + "=" * 70)
        print("EVALUATION REPORT (Transformer-Based)")
        print("=" * 70)

        print(f"\nModel: {self.model_name}")
        print(f"Total Samples: {metrics['total_samples']}")
        print(f"Overall Accuracy: {metrics['accuracy']:.2%}")
        print(f"Macro F1-Score: {metrics['macro_f1']:.4f}")

        print("\n" + "-" * 70)
        print("Per-Class Metrics:")
        print("-" * 70)
        print(f"{'Class':<15} {'Precision':<12} {'Recall':<12} {'F1-Score':<12} {'Support':<10}")
        print("-" * 70)

        for label in ['ENTAILMENT', 'CONTRADICTION', 'NEUTRAL']:
            m = metrics['per_class'][label]
            print(f"{label:<15} {m['precision']:<12.4f} {m['recall']:<12.4f} {m['f1']:<12.4f} {m['support']:<10}")

        print("\n" + "-" * 70)
        print("Confusion Matrix (rows=actual, cols=predicted):")
        print("-" * 70)
        labels = ['ENTAILMENT', 'CONTRADICTION', 'NEUTRAL']
        header = f"{'Actual/Pred':<15}" + "".join(f"{l[:6]:<12}" for l in labels)
        print(header)

        for actual in labels:
            row = f"{actual[:12]:<15}"
            for pred in labels:
                count = metrics['confusion_matrix'].get(actual, {}).get(pred, 0)
                row += f"{count:<12}"
            print(row)

        print("=" * 70 + "\n")


def main():
    """Demo of the transformer-based recognizer."""
    print("=" * 70)
    print("TRANSFORMER-BASED TEXTUAL ENTAILMENT RECOGNIZER (Phase 3)")
    print("=" * 70)

    recognizer = TransformerEntailmentRecognizer()

    # Test cases
    test_cases = [
        {
            "premise": "A man is playing a guitar.",
            "hypothesis": "A person is playing a musical instrument.",
            "expected": "ENTAILMENT"
        },
        {
            "premise": "The boy is happy.",
            "hypothesis": "The boy is sad and upset.",
            "expected": "CONTRADICTION"
        },
        {
            "premise": "It is raining outside.",
            "hypothesis": "The weather is sunny and clear.",
            "expected": "CONTRADICTION"
        },
        {
            "premise": "A teacher is explaining a lesson.",
            "hypothesis": "Someone is teaching.",
            "expected": "ENTAILMENT"
        },
        {
            "premise": "The store is closed.",
            "hypothesis": "The store is open for business.",
            "expected": "CONTRADICTION"
        },
        {
            "premise": "A man is walking down the street.",
            "hypothesis": "The man is going to work.",
            "expected": "NEUTRAL"
        },
        {
            "premise": "Two dogs are playing with a ball.",
            "hypothesis": "Animals are playing.",
            "expected": "ENTAILMENT"
        },
    ]

    print("\nTesting transformer model:\n")
    print("-" * 70)

    correct = 0
    for i, test in enumerate(test_cases, 1):
        label, confidence = recognizer.recognize_with_confidence(
            test['premise'], test['hypothesis']
        )

        status = "✓" if label == test['expected'] else "✗"
        if label == test['expected']:
            correct += 1

        print(f"Test {i}: {status}")
        print(f"  Premise:    {test['premise']}")
        print(f"  Hypothesis: {test['hypothesis']}")
        print(f"  Expected:   {test['expected']}")
        print(f"  Got:        {label} (confidence: {confidence})")
        print()

    print("-" * 70)
    print(f"Accuracy: {correct}/{len(test_cases)} ({correct/len(test_cases)*100:.1f}%)")
    print("=" * 70)


if __name__ == "__main__":
    main()
