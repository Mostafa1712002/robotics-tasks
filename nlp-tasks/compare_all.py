"""
Compare All Three Approaches for Textual Entailment Recognition

This script runs all three approaches on the same dataset:
1. Rule-Based (NLTK + WordNet)
2. Embedding-Based (Sentence Transformers)
3. Transformer-Based (DeBERTa-NLI)

Usage:
    python3 compare_all.py
"""

from index import TextualEntailmentRecognizer
from embedding_recognizer import EmbeddingEntailmentRecognizer
from transformer_recognizer import TransformerEntailmentRecognizer
from evaluate_snli import create_mini_dataset
import time


def main():
    print("=" * 70)
    print("FULL COMPARISON: All Three RTE Approaches")
    print("=" * 70)

    # Load mini dataset
    print("\nLoading mini dataset...")
    pairs, labels = create_mini_dataset()
    print(f"Loaded {len(pairs)} samples\n")

    results = {}

    # 1. Rule-Based
    print("-" * 70)
    print("[1/3] RULE-BASED APPROACH (NLTK + WordNet)")
    print("-" * 70)
    start = time.time()
    rule_based = TextualEntailmentRecognizer()
    rule_metrics = rule_based.evaluate(pairs, labels)
    rule_time = time.time() - start
    results['rule_based'] = {
        'metrics': rule_metrics,
        'time': rule_time
    }
    print(f"Accuracy: {rule_metrics['accuracy']:.2%}")
    print(f"Macro F1: {rule_metrics['macro_f1']:.4f}")
    print(f"Time: {rule_time:.2f}s\n")

    # 2. Embedding-Based
    print("-" * 70)
    print("[2/3] EMBEDDING-BASED APPROACH (Sentence Transformers)")
    print("-" * 70)
    start = time.time()
    embedding_based = EmbeddingEntailmentRecognizer()
    emb_metrics = embedding_based.evaluate(pairs, labels)
    emb_time = time.time() - start
    results['embedding'] = {
        'metrics': emb_metrics,
        'time': emb_time
    }
    print(f"Accuracy: {emb_metrics['accuracy']:.2%}")
    print(f"Macro F1: {emb_metrics['macro_f1']:.4f}")
    print(f"Time: {emb_time:.2f}s\n")

    # 3. Transformer-Based
    print("-" * 70)
    print("[3/3] TRANSFORMER-BASED APPROACH (DeBERTa-NLI)")
    print("-" * 70)
    start = time.time()
    transformer_based = TransformerEntailmentRecognizer()
    trans_metrics = transformer_based.evaluate(pairs, labels)
    trans_time = time.time() - start
    results['transformer'] = {
        'metrics': trans_metrics,
        'time': trans_time
    }
    print(f"Accuracy: {trans_metrics['accuracy']:.2%}")
    print(f"Macro F1: {trans_metrics['macro_f1']:.4f}")
    print(f"Time: {trans_time:.2f}s\n")

    # Summary Table
    print("\n" + "=" * 70)
    print("SUMMARY COMPARISON")
    print("=" * 70)

    print(f"\n{'Approach':<25} {'Accuracy':<12} {'Macro F1':<12} {'Time':<10}")
    print("-" * 60)
    print(f"{'Rule-Based (NLTK)':<25} {rule_metrics['accuracy']:.2%}{'':<5} {rule_metrics['macro_f1']:.4f}{'':<5} {rule_time:.2f}s")
    print(f"{'Embedding (ST)':<25} {emb_metrics['accuracy']:.2%}{'':<5} {emb_metrics['macro_f1']:.4f}{'':<5} {emb_time:.2f}s")
    print(f"{'Transformer (DeBERTa)':<25} {trans_metrics['accuracy']:.2%}{'':<5} {trans_metrics['macro_f1']:.4f}{'':<5} {trans_time:.2f}s")

    # Per-class comparison
    print("\n" + "-" * 70)
    print("PER-CLASS F1 SCORES")
    print("-" * 70)
    print(f"{'Class':<15} {'Rule-Based':<15} {'Embedding':<15} {'Transformer':<15}")
    print("-" * 60)

    for label in ['ENTAILMENT', 'CONTRADICTION', 'NEUTRAL']:
        rule_f1 = rule_metrics['per_class'][label]['f1']
        emb_f1 = emb_metrics['per_class'][label]['f1']
        trans_f1 = trans_metrics['per_class'][label]['f1']
        print(f"{label:<15} {rule_f1:.4f}{'':<9} {emb_f1:.4f}{'':<9} {trans_f1:.4f}")

    # Improvement summary
    print("\n" + "-" * 70)
    print("IMPROVEMENT FROM BASELINE (Rule-Based)")
    print("-" * 70)

    emb_acc_imp = emb_metrics['accuracy'] - rule_metrics['accuracy']
    trans_acc_imp = trans_metrics['accuracy'] - rule_metrics['accuracy']
    emb_f1_imp = emb_metrics['macro_f1'] - rule_metrics['macro_f1']
    trans_f1_imp = trans_metrics['macro_f1'] - rule_metrics['macro_f1']

    print(f"{'Metric':<20} {'Embedding':<20} {'Transformer':<20}")
    print("-" * 60)
    print(f"{'Accuracy':<20} {emb_acc_imp:+.2%}{'':<13} {trans_acc_imp:+.2%}")
    print(f"{'Macro F1':<20} {emb_f1_imp:+.4f}{'':<13} {trans_f1_imp:+.4f}")

    # Conclusion
    print("\n" + "=" * 70)
    print("CONCLUSION")
    print("=" * 70)
    print(f"""
Phase 1 (Rule-Based):
  - Simple heuristics with word overlap and negation detection
  - Fast but limited semantic understanding
  - Accuracy: {rule_metrics['accuracy']:.2%}

Phase 2 (Embedding-Based):
  - Sentence embeddings capture semantic similarity
  - Good balance of speed and accuracy
  - Accuracy: {emb_metrics['accuracy']:.2%} (+{emb_acc_imp:.2%})

Phase 3 (Transformer-Based):
  - Pre-trained on millions of NLI examples
  - State-of-the-art performance
  - Accuracy: {trans_metrics['accuracy']:.2%} (+{trans_acc_imp:.2%})

Key Takeaways:
  - Transformer model provides {trans_acc_imp:.2%} improvement over baseline
  - Best for production use where accuracy is critical
  - Embedding model is good middle-ground for resource-constrained environments
""")
    print("=" * 70)

    return results


if __name__ == "__main__":
    main()
