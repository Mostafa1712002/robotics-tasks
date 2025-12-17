"""
Embedding-Based Textual Entailment Recognition System

This module implements an advanced textual entailment recognition system using
sentence embeddings for semantic understanding. It dramatically improves upon
the rule-based approach by capturing deep semantic relationships.

Key improvements over rule-based:
- Understands "guitar" → "musical instrument" (hypernym relations)
- Detects "happy" vs "sad" as semantic opposites
- Captures paraphrase similarity
- Better handles synonyms and related concepts

Author: mostafa1712002
License: MIT
"""

import numpy as np
from collections import Counter

# Try to import sentence-transformers, fall back gracefully
try:
    from sentence_transformers import SentenceTransformer
    EMBEDDINGS_AVAILABLE = True
except ImportError:
    EMBEDDINGS_AVAILABLE = False
    print("Warning: sentence-transformers not installed.")
    print("Install with: pip install sentence-transformers")

# Import the original recognizer for hybrid approach
from index import TextualEntailmentRecognizer


class EmbeddingEntailmentRecognizer:
    """
    Advanced textual entailment recognizer using sentence embeddings.

    This class uses pre-trained sentence transformers to capture semantic
    similarity between premise and hypothesis. It combines embedding-based
    similarity with rule-based features for robust classification.

    Models available (trade-off between speed and accuracy):
    - 'all-MiniLM-L6-v2': Fast, good quality (default)
    - 'all-mpnet-base-v2': Slower, better quality
    - 'paraphrase-MiniLM-L6-v2': Optimized for paraphrase detection

    Classification approach:
    1. Compute semantic similarity using cosine similarity of embeddings
    2. Detect contradiction signals (negation, antonyms)
    3. Combine signals for final classification
    """

    def __init__(self, model_name='all-MiniLM-L6-v2'):
        """
        Initialize the embedding-based recognizer.

        Args:
            model_name (str): Name of the sentence-transformer model to use.
                            Default is 'all-MiniLM-L6-v2' for good speed/quality balance.
        """
        if not EMBEDDINGS_AVAILABLE:
            raise ImportError(
                "sentence-transformers is required. "
                "Install with: pip install sentence-transformers"
            )

        print(f"Loading model: {model_name}...")
        self.model = SentenceTransformer(model_name)
        self.model_name = model_name

        # Initialize rule-based recognizer for hybrid features
        self.rule_based = TextualEntailmentRecognizer()

        # Contradiction indicators (words that often signal contradiction)
        self.contradiction_pairs = {
            ('happy', 'sad'), ('sad', 'happy'),
            ('open', 'closed'), ('closed', 'open'),
            ('hot', 'cold'), ('cold', 'hot'),
            ('up', 'down'), ('down', 'up'),
            ('start', 'end'), ('end', 'start'),
            ('begin', 'finish'), ('finish', 'begin'),
            ('love', 'hate'), ('hate', 'love'),
            ('good', 'bad'), ('bad', 'good'),
            ('alive', 'dead'), ('dead', 'alive'),
            ('empty', 'full'), ('full', 'empty'),
            ('inside', 'outside'), ('outside', 'inside'),
            ('light', 'dark'), ('dark', 'light'),
            ('young', 'old'), ('old', 'young'),
            ('fast', 'slow'), ('slow', 'fast'),
            ('loud', 'quiet'), ('quiet', 'loud'),
            ('win', 'lose'), ('lose', 'win'),
            ('buy', 'sell'), ('sell', 'buy'),
            ('push', 'pull'), ('pull', 'push'),
            ('arrive', 'leave'), ('leave', 'arrive'),
            ('asleep', 'awake'), ('awake', 'asleep'),
            ('sit', 'stand'), ('stand', 'sit'),
            ('laugh', 'cry'), ('cry', 'laugh'),
            ('positive', 'negative'), ('negative', 'positive'),
            ('success', 'failure'), ('failure', 'success'),
            ('boring', 'exciting'), ('exciting', 'boring'),
        }

        # Intensifiers that strengthen contradiction signals
        self.intensifiers = {'very', 'extremely', 'completely', 'totally', 'absolutely'}

        print("Model loaded successfully!")

    def get_embedding(self, text):
        """
        Get sentence embedding for a text.

        Args:
            text (str): Input text

        Returns:
            numpy.ndarray: Embedding vector
        """
        return self.model.encode(text, convert_to_numpy=True)

    def cosine_similarity(self, vec1, vec2):
        """
        Calculate cosine similarity between two vectors.

        Cosine similarity measures the angle between vectors:
        - 1.0 = identical direction (same meaning)
        - 0.0 = orthogonal (unrelated)
        - -1.0 = opposite direction (opposite meaning)

        Args:
            vec1, vec2: Numpy arrays

        Returns:
            float: Cosine similarity score
        """
        dot_product = np.dot(vec1, vec2)
        norm1 = np.linalg.norm(vec1)
        norm2 = np.linalg.norm(vec2)

        if norm1 == 0 or norm2 == 0:
            return 0.0

        return dot_product / (norm1 * norm2)

    def detect_antonym_pairs(self, premise, hypothesis):
        """
        Detect antonym pairs between premise and hypothesis.

        Checks if words in premise have antonyms in hypothesis,
        which is a strong signal for contradiction.

        Args:
            premise (str): Premise text
            hypothesis (str): Hypothesis text

        Returns:
            list: List of detected antonym pairs
        """
        premise_tokens = set(self.rule_based.preprocess(premise))
        hypothesis_tokens = set(self.rule_based.preprocess(hypothesis))

        detected_pairs = []

        for p_word in premise_tokens:
            for h_word in hypothesis_tokens:
                if (p_word, h_word) in self.contradiction_pairs:
                    detected_pairs.append((p_word, h_word))

        return detected_pairs

    def compute_contradiction_score(self, premise, hypothesis, similarity):
        """
        Compute contradiction score based on multiple signals.

        Combines:
        1. Negation mismatch (from rule-based)
        2. Antonym detection
        3. Similarity patterns (high similarity + negation = contradiction)

        Args:
            premise (str): Premise text
            hypothesis (str): Hypothesis text
            similarity (float): Semantic similarity score

        Returns:
            float: Contradiction score (0.0 to 1.0)
        """
        score = 0.0

        # Check negation mismatch
        premise_negated = self.rule_based.negation_check(premise)
        hypothesis_negated = self.rule_based.negation_check(hypothesis)

        if premise_negated != hypothesis_negated:
            # Negation mismatch with high similarity = strong contradiction
            if similarity > 0.5:
                score += 0.6
            else:
                score += 0.3

        # Check for antonym pairs
        antonym_pairs = self.detect_antonym_pairs(premise, hypothesis)
        if antonym_pairs:
            score += 0.4 * min(len(antonym_pairs), 2)  # Cap at 2 pairs

        # High similarity but moderate contradiction signals
        # Often indicates paraphrases with opposite meaning
        if similarity > 0.7 and (premise_negated != hypothesis_negated or antonym_pairs):
            score += 0.2

        return min(score, 1.0)  # Cap at 1.0

    def recognize(self, premise, hypothesis):
        """
        Recognize textual entailment using embeddings.

        Classification logic:
        1. Compute semantic similarity using sentence embeddings
        2. Check for contradiction signals (negation, antonyms)
        3. Apply decision thresholds

        Args:
            premise (str): The premise text
            hypothesis (str): The hypothesis text

        Returns:
            str: 'ENTAILMENT', 'CONTRADICTION', or 'NEUTRAL'
        """
        # Get embeddings
        premise_emb = self.get_embedding(premise)
        hypothesis_emb = self.get_embedding(hypothesis)

        # Compute semantic similarity
        similarity = self.cosine_similarity(premise_emb, hypothesis_emb)

        # Compute contradiction score
        contradiction_score = self.compute_contradiction_score(
            premise, hypothesis, similarity
        )

        # Decision logic
        if contradiction_score > 0.5:
            return "CONTRADICTION"
        elif similarity > 0.6:
            # High similarity without contradiction signals = entailment
            return "ENTAILMENT"
        elif similarity > 0.35:
            # Moderate similarity = could be weak entailment or neutral
            # Use additional context
            if contradiction_score > 0.2:
                return "CONTRADICTION"
            elif similarity > 0.5:
                return "ENTAILMENT"
            else:
                return "NEUTRAL"
        else:
            # Low similarity = neutral (unrelated)
            return "NEUTRAL"

    def recognize_with_confidence(self, premise, hypothesis):
        """
        Recognize textual entailment with confidence score.

        Args:
            premise (str): The premise text
            hypothesis (str): The hypothesis text

        Returns:
            tuple: (label, confidence_score)
        """
        # Get embeddings
        premise_emb = self.get_embedding(premise)
        hypothesis_emb = self.get_embedding(hypothesis)

        # Compute scores
        similarity = self.cosine_similarity(premise_emb, hypothesis_emb)
        contradiction_score = self.compute_contradiction_score(
            premise, hypothesis, similarity
        )

        # Get label
        label = self.recognize(premise, hypothesis)

        # Compute confidence based on label
        if label == "ENTAILMENT":
            # Confidence based on how high similarity is above threshold
            confidence = min(similarity, 0.95)
        elif label == "CONTRADICTION":
            # Confidence based on contradiction score
            confidence = min(contradiction_score + 0.3, 0.95)
        else:  # NEUTRAL
            # Confidence based on how far from other thresholds
            confidence = max(0.4, 1.0 - similarity)

        return label, round(confidence, 3)

    def recognize_batch(self, pairs):
        """
        Batch processing for multiple premise-hypothesis pairs.

        Efficiently processes multiple pairs using batch encoding.

        Args:
            pairs (list): List of {'premise': str, 'hypothesis': str} dicts

        Returns:
            list: List of predicted labels
        """
        # Extract texts
        premises = [p['premise'] for p in pairs]
        hypotheses = [p['hypothesis'] for p in pairs]

        # Batch encode for efficiency
        premise_embeddings = self.model.encode(premises, convert_to_numpy=True)
        hypothesis_embeddings = self.model.encode(hypotheses, convert_to_numpy=True)

        results = []
        for i, pair in enumerate(pairs):
            similarity = self.cosine_similarity(
                premise_embeddings[i],
                hypothesis_embeddings[i]
            )
            contradiction_score = self.compute_contradiction_score(
                pair['premise'], pair['hypothesis'], similarity
            )

            # Apply decision logic
            if contradiction_score > 0.5:
                label = "CONTRADICTION"
            elif similarity > 0.6:
                label = "ENTAILMENT"
            elif similarity > 0.35:
                if contradiction_score > 0.2:
                    label = "CONTRADICTION"
                elif similarity > 0.5:
                    label = "ENTAILMENT"
                else:
                    label = "NEUTRAL"
            else:
                label = "NEUTRAL"

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
        # Get predictions
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
        print("EVALUATION REPORT (Embedding-Based)")
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


def compare_approaches(pairs, labels):
    """
    Compare rule-based vs embedding-based approaches.

    Args:
        pairs: List of premise-hypothesis pairs
        labels: List of gold labels
    """
    print("\n" + "=" * 70)
    print("COMPARISON: Rule-Based vs Embedding-Based")
    print("=" * 70)

    # Rule-based
    print("\n[1] Rule-Based Approach")
    print("-" * 70)
    rule_based = TextualEntailmentRecognizer()
    rule_metrics = rule_based.evaluate(pairs, labels)
    print(f"Accuracy: {rule_metrics['accuracy']:.2%}")
    print(f"Macro F1: {rule_metrics['macro_f1']:.4f}")

    # Embedding-based
    print("\n[2] Embedding-Based Approach")
    print("-" * 70)
    embedding_based = EmbeddingEntailmentRecognizer()
    emb_metrics = embedding_based.evaluate(pairs, labels)
    print(f"Accuracy: {emb_metrics['accuracy']:.2%}")
    print(f"Macro F1: {emb_metrics['macro_f1']:.4f}")

    # Improvement
    print("\n" + "-" * 70)
    print("IMPROVEMENT:")
    print("-" * 70)
    acc_improvement = emb_metrics['accuracy'] - rule_metrics['accuracy']
    f1_improvement = emb_metrics['macro_f1'] - rule_metrics['macro_f1']
    print(f"Accuracy: {acc_improvement:+.2%}")
    print(f"Macro F1: {f1_improvement:+.4f}")

    print("\nPer-Class F1 Comparison:")
    for label in ['ENTAILMENT', 'CONTRADICTION', 'NEUTRAL']:
        rule_f1 = rule_metrics['per_class'][label]['f1']
        emb_f1 = emb_metrics['per_class'][label]['f1']
        improvement = emb_f1 - rule_f1
        print(f"  {label}: {rule_f1:.4f} → {emb_f1:.4f} ({improvement:+.4f})")

    print("=" * 70)

    return rule_metrics, emb_metrics


def main():
    """Demo of the embedding-based recognizer."""
    print("=" * 70)
    print("EMBEDDING-BASED TEXTUAL ENTAILMENT RECOGNIZER")
    print("=" * 70)

    # Initialize
    recognizer = EmbeddingEntailmentRecognizer()

    # Test cases that failed with rule-based
    test_cases = [
        # These failed with rule-based (no word overlap)
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
            "premise": "The bird is flying.",
            "hypothesis": "An animal is moving through the air.",
            "expected": "ENTAILMENT"
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
    ]

    print("\nTesting cases that failed with rule-based approach:\n")
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
    print(f"Accuracy on difficult cases: {correct}/{len(test_cases)} ({correct/len(test_cases)*100:.1f}%)")
    print("=" * 70)


if __name__ == "__main__":
    main()
