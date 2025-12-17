"""
Compare Rule-Based vs Embedding-Based Textual Entailment Recognition

This script runs both approaches on the same dataset and shows
the improvement from using sentence embeddings.

Usage:
    python3 compare_approaches.py
"""

from index import TextualEntailmentRecognizer
from embedding_recognizer import EmbeddingEntailmentRecognizer
from evaluate_snli import create_mini_dataset


def main():
    print("=" * 70)
    print("COMPARISON: Rule-Based vs Embedding-Based RTE")
    print("=" * 70)

    # Load mini dataset
    print("\nLoading mini dataset...")
    pairs, labels = create_mini_dataset()
    print(f"Loaded {len(pairs)} samples\n")

    # Initialize recognizers
    print("Initializing Rule-Based Recognizer...")
    rule_based = TextualEntailmentRecognizer()

    print("Initializing Embedding-Based Recognizer...")
    embedding_based = EmbeddingEntailmentRecognizer()

    # Evaluate Rule-Based
    print("\n" + "=" * 70)
    print("RULE-BASED APPROACH")
    print("=" * 70)
    rule_metrics = rule_based.evaluate(pairs, labels)
    rule_based.print_evaluation_report(rule_metrics)

    # Evaluate Embedding-Based
    print("\n" + "=" * 70)
    print("EMBEDDING-BASED APPROACH")
    print("=" * 70)
    emb_metrics = embedding_based.evaluate(pairs, labels)
    embedding_based.print_evaluation_report(emb_metrics)

    # Comparison Summary
    print("\n" + "=" * 70)
    print("IMPROVEMENT SUMMARY")
    print("=" * 70)

    acc_improvement = emb_metrics['accuracy'] - rule_metrics['accuracy']
    f1_improvement = emb_metrics['macro_f1'] - rule_metrics['macro_f1']

    print(f"\n{'Metric':<20} {'Rule-Based':<15} {'Embedding':<15} {'Improvement':<15}")
    print("-" * 65)
    print(f"{'Accuracy':<20} {rule_metrics['accuracy']:.2%}{'':<8} {emb_metrics['accuracy']:.2%}{'':<8} {acc_improvement:+.2%}")
    print(f"{'Macro F1':<20} {rule_metrics['macro_f1']:.4f}{'':<8} {emb_metrics['macro_f1']:.4f}{'':<8} {f1_improvement:+.4f}")

    print("\nPer-Class F1 Comparison:")
    print("-" * 65)
    for label in ['ENTAILMENT', 'CONTRADICTION', 'NEUTRAL']:
        rule_f1 = rule_metrics['per_class'][label]['f1']
        emb_f1 = emb_metrics['per_class'][label]['f1']
        improvement = emb_f1 - rule_f1
        print(f"  {label:<15} {rule_f1:.4f} → {emb_f1:.4f} ({improvement:+.4f})")

    print("\n" + "=" * 70)
    print("CONCLUSION")
    print("=" * 70)
    if acc_improvement > 0:
        print(f"\n✓ Embedding-based approach improved accuracy by {acc_improvement:.2%}")
        print(f"✓ Macro F1 improved by {f1_improvement:.4f}")
    else:
        print(f"\n✗ No improvement (may need threshold tuning)")

    print("\nKey improvements:")
    print("  - Better semantic understanding (guitar → musical instrument)")
    print("  - Captures paraphrase similarity")
    print("  - Handles synonyms without explicit dictionary")
    print("\nAreas for further improvement:")
    print("  - Contradiction detection (antonyms need better handling)")
    print("  - Threshold tuning for better precision/recall balance")
    print("  - Phase 3: Fine-tuned BERT for optimal performance")
    print("=" * 70)


if __name__ == "__main__":
    main()
