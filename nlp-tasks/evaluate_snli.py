"""
SNLI Dataset Evaluation Script

This script evaluates the Textual Entailment Recognition System against
the Stanford Natural Language Inference (SNLI) dataset.

SNLI Dataset:
- 570K human-written sentence pairs
- Labels: entailment, contradiction, neutral
- Standard benchmark for NLI systems

Usage:
    python evaluate_snli.py [--samples N] [--download]

Options:
    --samples N   : Number of samples to evaluate (default: 1000)
    --download    : Download SNLI dataset if not present
    --full        : Run on full test set (~10K samples)

Author: mostafa1712002
License: MIT
"""

import json
import os
import sys
import random
import urllib.request
import zipfile
from index import TextualEntailmentRecognizer

# SNLI dataset configuration
SNLI_URL = "https://nlp.stanford.edu/projects/snli/snli_1.0.zip"
SNLI_DIR = "data/snli_1.0"
SNLI_TEST_FILE = "data/snli_1.0/snli_1.0_test.jsonl"
SNLI_DEV_FILE = "data/snli_1.0/snli_1.0_dev.jsonl"


def download_snli():
    """
    Download and extract SNLI dataset.

    Downloads the official SNLI 1.0 dataset from Stanford NLP.
    The dataset is ~100MB compressed, ~200MB uncompressed.
    """
    # Create data directory
    os.makedirs("data", exist_ok=True)

    zip_path = "data/snli_1.0.zip"

    # Check if already downloaded
    if os.path.exists(SNLI_DIR):
        print("SNLI dataset already exists.")
        return True

    print("Downloading SNLI dataset (~100MB)...")
    print(f"URL: {SNLI_URL}")

    try:
        # Download with progress
        def progress_hook(count, block_size, total_size):
            percent = int(count * block_size * 100 / total_size)
            sys.stdout.write(f"\rDownloading: {percent}%")
            sys.stdout.flush()

        urllib.request.urlretrieve(SNLI_URL, zip_path, progress_hook)
        print("\nDownload complete!")

        # Extract
        print("Extracting...")
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            zip_ref.extractall("data")

        # Clean up zip file
        os.remove(zip_path)
        print("SNLI dataset ready!")
        return True

    except Exception as e:
        print(f"\nError downloading SNLI: {e}")
        print("\nAlternative: Download manually from:")
        print("  https://nlp.stanford.edu/projects/snli/")
        print("  Extract to: data/snli_1.0/")
        return False


def load_snli_data(file_path, max_samples=None):
    """
    Load SNLI data from JSONL file.

    Args:
        file_path (str): Path to SNLI JSONL file
        max_samples (int): Maximum number of samples to load (None for all)

    Returns:
        tuple: (pairs, labels) where:
            - pairs: List of {'premise': str, 'hypothesis': str}
            - labels: List of gold labels (ENTAILMENT, CONTRADICTION, NEUTRAL)
    """
    pairs = []
    labels = []

    # Label mapping from SNLI format to our format
    label_map = {
        'entailment': 'ENTAILMENT',
        'contradiction': 'CONTRADICTION',
        'neutral': 'NEUTRAL'
    }

    with open(file_path, 'r', encoding='utf-8') as f:
        for i, line in enumerate(f):
            if max_samples and i >= max_samples:
                break

            data = json.loads(line)

            # Skip samples with '-' label (no gold label)
            gold_label = data.get('gold_label', '-')
            if gold_label == '-':
                continue

            # Map label to our format
            if gold_label in label_map:
                pairs.append({
                    'premise': data['sentence1'],
                    'hypothesis': data['sentence2']
                })
                labels.append(label_map[gold_label])

    return pairs, labels


def create_mini_dataset():
    """
    Create a mini dataset for testing without downloading full SNLI.

    This provides 100 manually curated examples representing
    typical SNLI-style sentence pairs for quick evaluation.

    Returns:
        tuple: (pairs, labels)
    """
    # Curated examples based on SNLI patterns
    examples = [
        # ENTAILMENT examples (hypothesis follows from premise)
        ("A man is playing a guitar.", "A person is playing a musical instrument.", "ENTAILMENT"),
        ("Children are running in the park.", "Kids are outside.", "ENTAILMENT"),
        ("A woman is cooking dinner in the kitchen.", "Someone is preparing food.", "ENTAILMENT"),
        ("Two dogs are playing with a ball.", "Animals are playing.", "ENTAILMENT"),
        ("A soccer game with multiple males playing.", "Some men are playing a sport.", "ENTAILMENT"),
        ("The boy is eating an apple.", "A child is eating fruit.", "ENTAILMENT"),
        ("A group of friends are having a party.", "People are celebrating.", "ENTAILMENT"),
        ("The cat is sleeping on the couch.", "An animal is resting.", "ENTAILMENT"),
        ("Students are studying in the library.", "People are reading.", "ENTAILMENT"),
        ("A chef is preparing a meal.", "Someone is cooking.", "ENTAILMENT"),
        ("The basketball players are on the court.", "Athletes are playing.", "ENTAILMENT"),
        ("A mother is holding her baby.", "A woman has a child.", "ENTAILMENT"),
        ("Musicians are performing on stage.", "People are playing music.", "ENTAILMENT"),
        ("The airplane is flying in the sky.", "A vehicle is in the air.", "ENTAILMENT"),
        ("Workers are building a house.", "People are constructing something.", "ENTAILMENT"),
        ("A teacher is explaining a lesson.", "Someone is teaching.", "ENTAILMENT"),
        ("The swimmer is in the pool.", "A person is in water.", "ENTAILMENT"),
        ("A couple is dancing at their wedding.", "People are celebrating.", "ENTAILMENT"),
        ("The doctor is examining a patient.", "A medical professional is working.", "ENTAILMENT"),
        ("Children are playing video games.", "Kids are having fun.", "ENTAILMENT"),

        # CONTRADICTION examples (hypothesis contradicts premise)
        ("The man is sleeping.", "The man is running.", "CONTRADICTION"),
        ("It is raining outside.", "The weather is sunny and clear.", "CONTRADICTION"),
        ("The store is closed.", "The store is open for business.", "CONTRADICTION"),
        ("She is sitting down.", "She is standing up.", "CONTRADICTION"),
        ("The cat is sitting on the mat.", "The cat is not sitting on the mat.", "CONTRADICTION"),
        ("The children are laughing.", "The children are crying.", "CONTRADICTION"),
        ("He is eating dinner.", "He is sleeping in bed.", "CONTRADICTION"),
        ("The car is moving.", "The car is parked.", "CONTRADICTION"),
        ("People are inside the building.", "Everyone is outside.", "CONTRADICTION"),
        ("The light is on.", "The room is completely dark.", "CONTRADICTION"),
        ("The boy is happy.", "The boy is sad and upset.", "CONTRADICTION"),
        ("She is wearing a red dress.", "She is wearing blue pants.", "CONTRADICTION"),
        ("The game is starting.", "The game has ended.", "CONTRADICTION"),
        ("He loves pizza.", "He hates pizza.", "CONTRADICTION"),
        ("The dog is barking.", "The dog is silent.", "CONTRADICTION"),
        ("She is young.", "She is elderly.", "CONTRADICTION"),
        ("The water is hot.", "The water is freezing cold.", "CONTRADICTION"),
        ("He arrived early.", "He arrived late.", "CONTRADICTION"),
        ("The movie is boring.", "The movie is exciting.", "CONTRADICTION"),
        ("She is alone.", "She is with many people.", "CONTRADICTION"),

        # NEUTRAL examples (hypothesis neither follows nor contradicts)
        ("A man is walking down the street.", "The man is going to work.", "NEUTRAL"),
        ("A woman is reading a book.", "The book is interesting.", "NEUTRAL"),
        ("Children are playing in the yard.", "They are playing soccer.", "NEUTRAL"),
        ("A person on a horse jumps over a broken down airplane.", "A person is training his horse for a competition.", "NEUTRAL"),
        ("Children smiling and waving at camera.", "They are smiling at their parents.", "NEUTRAL"),
        ("A boy is jumping on skateboard in the middle of a red bridge.", "The boy skates down the sidewalk.", "NEUTRAL"),
        ("A black race car starts up in front of a crowd of people.", "A man is driving down a lonely road.", "NEUTRAL"),
        ("A dog is running in the field.", "The dog is chasing a rabbit.", "NEUTRAL"),
        ("People are sitting at a table.", "They are having a business meeting.", "NEUTRAL"),
        ("A woman is looking at her phone.", "She is reading a text from her friend.", "NEUTRAL"),
        ("The man is wearing a hat.", "He bought the hat yesterday.", "NEUTRAL"),
        ("A group is standing outside.", "They are waiting for a bus.", "NEUTRAL"),
        ("Someone is playing piano.", "They are performing at a concert.", "NEUTRAL"),
        ("A car is parked on the street.", "The car belongs to a doctor.", "NEUTRAL"),
        ("People are eating at a restaurant.", "The food is delicious.", "NEUTRAL"),
        ("A child is drawing.", "The child is talented.", "NEUTRAL"),
        ("Workers are in an office.", "They are working on a big project.", "NEUTRAL"),
        ("A woman is walking her dog.", "She walks the dog every morning.", "NEUTRAL"),
        ("Students are in a classroom.", "They are taking a test.", "NEUTRAL"),
        ("A man is fishing by the lake.", "He catches a big fish.", "NEUTRAL"),

        # Additional mixed examples
        ("Two people are hugging.", "Friends are greeting each other.", "ENTAILMENT"),
        ("The bird is flying.", "An animal is moving through the air.", "ENTAILMENT"),
        ("A fire is burning.", "Something is hot.", "ENTAILMENT"),
        ("The baby is crying.", "The baby is laughing.", "CONTRADICTION"),
        ("He is asleep.", "He is wide awake.", "CONTRADICTION"),
        ("The door is open.", "The door is closed.", "CONTRADICTION"),
        ("A man is jogging.", "He is training for a marathon.", "NEUTRAL"),
        ("She is cooking.", "She is making pasta.", "NEUTRAL"),
        ("The phone is ringing.", "Someone is calling about work.", "NEUTRAL"),
        ("A cat is on the roof.", "The cat climbed up there.", "NEUTRAL"),

        # More challenging examples
        ("Several people are waiting at a bus stop.", "People are commuting.", "ENTAILMENT"),
        ("A musician strums a guitar.", "Someone is making music.", "ENTAILMENT"),
        ("The athlete crosses the finish line first.", "A runner wins the race.", "ENTAILMENT"),
        ("Nobody is in the room.", "The room is full of people.", "CONTRADICTION"),
        ("The movie theater is empty.", "The theater is packed with viewers.", "CONTRADICTION"),
        ("All the lights are off.", "The lights are brightly shining.", "CONTRADICTION"),
        ("A woman is grocery shopping.", "She is buying ingredients for a recipe.", "NEUTRAL"),
        ("The man is checking his watch.", "He is late for an appointment.", "NEUTRAL"),
        ("Two kids are building a sandcastle.", "They are at the beach.", "NEUTRAL"),
        ("An elderly couple is walking slowly.", "They have been married for 50 years.", "NEUTRAL"),
    ]

    # Shuffle for randomness
    random.shuffle(examples)

    pairs = [{'premise': p, 'hypothesis': h} for p, h, _ in examples]
    labels = [l for _, _, l in examples]

    return pairs, labels


def run_evaluation(pairs, labels, recognizer):
    """
    Run evaluation and print results.

    Args:
        pairs: List of premise-hypothesis pairs
        labels: List of gold labels
        recognizer: TextualEntailmentRecognizer instance

    Returns:
        dict: Evaluation metrics
    """
    print(f"\nEvaluating on {len(pairs)} samples...")
    print("-" * 70)

    # Run evaluation
    metrics = recognizer.evaluate(pairs, labels)

    # Print report
    recognizer.print_evaluation_report(metrics)

    return metrics


def analyze_errors(pairs, predictions, gold_labels, max_errors=10):
    """
    Analyze and display error cases.

    Args:
        pairs: List of premise-hypothesis pairs
        predictions: List of predicted labels
        gold_labels: List of gold labels
        max_errors: Maximum number of errors to display
    """
    print("\n" + "=" * 70)
    print("ERROR ANALYSIS")
    print("=" * 70)

    errors = []
    for i, (pair, pred, gold) in enumerate(zip(pairs, predictions, gold_labels)):
        if pred != gold:
            errors.append({
                'premise': pair['premise'],
                'hypothesis': pair['hypothesis'],
                'predicted': pred,
                'actual': gold
            })

    print(f"\nTotal errors: {len(errors)} / {len(pairs)} ({len(errors)/len(pairs)*100:.1f}%)")
    print(f"\nShowing first {min(max_errors, len(errors))} errors:\n")

    for i, err in enumerate(errors[:max_errors], 1):
        print(f"Error {i}:")
        print(f"  Premise:    {err['premise'][:70]}...")
        print(f"  Hypothesis: {err['hypothesis'][:70]}...")
        print(f"  Predicted:  {err['predicted']}")
        print(f"  Actual:     {err['actual']}")
        print()

    # Error breakdown by type
    print("-" * 70)
    print("Error breakdown by misclassification type:")
    print("-" * 70)

    from collections import Counter
    error_types = Counter((err['actual'], err['predicted']) for err in errors)

    for (actual, predicted), count in error_types.most_common():
        print(f"  {actual} → {predicted}: {count} errors")


def main():
    """Main function to run SNLI evaluation."""
    import argparse

    parser = argparse.ArgumentParser(description='Evaluate RTE system on SNLI dataset')
    parser.add_argument('--samples', type=int, default=100,
                        help='Number of samples to evaluate (default: 100)')
    parser.add_argument('--download', action='store_true',
                        help='Download SNLI dataset')
    parser.add_argument('--full', action='store_true',
                        help='Run on full test set')
    parser.add_argument('--mini', action='store_true',
                        help='Use mini dataset (no download required)')
    parser.add_argument('--errors', type=int, default=10,
                        help='Number of errors to show in analysis')

    args = parser.parse_args()

    print("=" * 70)
    print("SNLI DATASET EVALUATION")
    print("Textual Entailment Recognition System")
    print("=" * 70)

    # Initialize recognizer
    print("\nInitializing recognizer...")
    recognizer = TextualEntailmentRecognizer()

    # Determine data source
    if args.mini or not os.path.exists(SNLI_DIR):
        if args.download:
            success = download_snli()
            if not success:
                print("\nFalling back to mini dataset...")
                args.mini = True
        else:
            print("\nSNLI dataset not found. Using mini dataset.")
            print("(Use --download to download full SNLI dataset)")
            args.mini = True

    # Load data
    if args.mini:
        print("\nLoading mini dataset (100 curated examples)...")
        pairs, labels = create_mini_dataset()
        if args.samples and args.samples < len(pairs):
            pairs = pairs[:args.samples]
            labels = labels[:args.samples]
    else:
        # Use full SNLI
        test_file = SNLI_TEST_FILE if os.path.exists(SNLI_TEST_FILE) else SNLI_DEV_FILE

        max_samples = None if args.full else args.samples
        print(f"\nLoading SNLI data from {test_file}...")
        pairs, labels = load_snli_data(test_file, max_samples)

    print(f"Loaded {len(pairs)} samples")

    # Show label distribution
    from collections import Counter
    dist = Counter(labels)
    print(f"\nLabel distribution:")
    for label, count in dist.most_common():
        print(f"  {label}: {count} ({count/len(labels)*100:.1f}%)")

    # Run evaluation
    metrics = run_evaluation(pairs, labels, recognizer)

    # Error analysis
    analyze_errors(pairs, metrics['predictions'], labels, args.errors)

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"Dataset: {'Mini (curated)' if args.mini else 'SNLI'}")
    print(f"Samples: {len(pairs)}")
    print(f"Accuracy: {metrics['accuracy']:.2%}")
    print(f"Macro F1: {metrics['macro_f1']:.4f}")
    print("\nNote: This is a rule-based system. For better results,")
    print("consider Phase 2 (embeddings) or Phase 3 (deep learning).")
    print("=" * 70)


if __name__ == "__main__":
    main()
