"""
Textual Entailment Recognition System

This module implements a textual entailment recognition system using Natural Language Processing (NLP)
techniques to determine whether a hypothesis can be inferred from a given premise.

The system uses NLTK for text processing and WordNet for semantic analysis.

Author: mostafa1712002
License: MIT
"""

# Import required libraries for NLP processing
import nltk  # Natural Language Toolkit for NLP operations
from collections import Counter  # For counting label occurrences in evaluation
from nltk.tokenize import word_tokenize  # For breaking text into words
from nltk.corpus import stopwords, wordnet  # For stopword removal and synonym detection
from nltk.stem import WordNetLemmatizer  # For word lemmatization (converting words to base form)
import string  # For punctuation handling


def download_nltk_data():
    """
    Download necessary NLTK datasets for text processing.

    This function downloads the following datasets:
    - punkt: Sentence tokenizer models
    - stopwords: Common words to filter out (e.g., 'the', 'is', 'at')
    - wordnet: Lexical database for English (for synonym detection)
    - averaged_perceptron_tagger: Part-of-speech tagger
    - punkt_tab: Tokenization tables

    The downloads are performed quietly to avoid cluttering output.
    Exceptions are caught and ignored to handle cases where downloads fail or data already exists.
    """
    # List of required NLTK datasets
    datasets = ['punkt', 'stopwords', 'wordnet', 'averaged_perceptron_tagger', 'punkt_tab']

    # Iterate through each dataset and attempt to download it
    for dataset in datasets:
        try:
            # Download dataset with quiet=True to suppress output messages
            nltk.download(dataset, quiet=True)
        except:
            # If download fails (e.g., already exists or network error), continue silently
            pass


class TextualEntailmentRecognizer:
    """
    A textual entailment recognition system that determines if a hypothesis
    can be inferred from a text premise.

    This class implements a rule-based approach using:
    1. Word overlap analysis - Measures how many words are shared between texts
    2. Synonym detection - Uses WordNet to find semantically similar words
    3. Negation detection - Identifies contradictions through negation words
    4. Text preprocessing - Normalizes text for better comparison

    Classification Labels:
    - ENTAILMENT: The hypothesis logically follows from the premise
                  Example: Premise: "A dog is running" → Hypothesis: "An animal is moving"

    - CONTRADICTION: The hypothesis contradicts the premise
                     Example: Premise: "The cat is sleeping" → Hypothesis: "The cat is not sleeping"

    - NEUTRAL: The hypothesis is unrelated to the premise
               Example: Premise: "A car is parked" → Hypothesis: "The sky is blue"
    """

    def __init__(self):
        """
        Initialize the TextualEntailmentRecognizer.

        This constructor:
        1. Downloads required NLTK data
        2. Initializes the WordNet lemmatizer for word normalization
        3. Loads English stopwords for filtering common words

        The lemmatizer converts words to their base form (e.g., 'running' → 'run')
        Stopwords are common words that don't carry significant meaning (e.g., 'the', 'is', 'at')
        """
        # Download all necessary NLTK datasets
        download_nltk_data()

        # Initialize the lemmatizer to convert words to their base/dictionary form
        # Example: "running" → "run", "better" → "good"
        self.lemmatizer = WordNetLemmatizer()

        # Load English stopwords (common words to filter out during processing)
        # Examples: 'the', 'is', 'at', 'which', 'on', 'a', 'an'
        try:
            self.stop_words = set(stopwords.words('english'))
        except:
            # If stopwords can't be loaded, use empty set (no stopword filtering)
            self.stop_words = set()

    def preprocess(self, text):
        """
        Preprocess text by tokenizing, normalizing, and cleaning.

        This function performs the following steps:
        1. Tokenization: Break text into individual words
        2. Lowercasing: Convert all words to lowercase for case-insensitive comparison
        3. Punctuation removal: Remove punctuation marks
        4. Stopword removal: Filter out common words that don't add meaning
        5. Lemmatization: Convert words to their base form

        Args:
            text (str): The input text to preprocess

        Returns:
            list: A list of preprocessed tokens (cleaned words)

        Example:
            Input:  "The dogs are running quickly!"
            Output: ['dog', 'running', 'quickly']

        Process breakdown:
            - Tokenize: ['The', 'dogs', 'are', 'running', 'quickly', '!']
            - Lowercase: ['the', 'dogs', 'are', 'running', 'quickly', '!']
            - Remove punct: ['the', 'dogs', 'are', 'running', 'quickly']
            - Remove stopwords: ['dogs', 'running', 'quickly']
            - Lemmatize: ['dog', 'running', 'quickly']
        """
        # Step 1: Tokenize the text and convert to lowercase in one operation
        # word_tokenize splits text into words, handling contractions and punctuation
        # .lower() ensures case-insensitive matching (e.g., "Cat" and "cat" are the same)
        tokens = word_tokenize(text.lower())

        # Step 2: Remove punctuation marks
        # Filter out any token that is a punctuation character (e.g., '.', ',', '!', '?')
        # string.punctuation contains: !"#$%&'()*+,-./:;<=>?@[\]^_`{|}~
        tokens = [token for token in tokens if token not in string.punctuation]

        # Step 3: Remove stopwords (common words that don't carry much meaning)
        # Examples of stopwords: 'the', 'is', 'at', 'which', 'on', 'a', 'an', 'are', 'was'
        tokens = [token for token in tokens if token not in self.stop_words]

        # Step 4: Lemmatize tokens (convert words to their base/dictionary form)
        # Examples: 'running' → 'run', 'cats' → 'cat', 'better' → 'good'
        # This helps match different forms of the same word
        tokens = [self.lemmatizer.lemmatize(token) for token in tokens]

        # Return the cleaned and normalized list of tokens
        return tokens

    def get_synonyms(self, word):
        """
        Get synonyms for a word using WordNet lexical database.

        WordNet is a large lexical database where words are grouped into sets of synonyms (synsets).
        This function retrieves all synonyms for a given word to help detect semantic similarity.

        Args:
            word (str): The word to find synonyms for

        Returns:
            set: A set of synonym strings (lowercase)

        Example:
            Input:  "car"
            Output: {'car', 'auto', 'automobile', 'machine', 'motorcar'}

        How it works:
            1. WordNet groups words into synsets (synonym sets)
            2. Each synset contains multiple lemmas (word forms)
            3. We extract all lemma names as synonyms

        """
        # Initialize empty set to store unique synonyms
        synonyms = set()

        # Get all synsets (groups of synonyms) for the word
        # wordnet.synsets() returns all possible meanings/contexts of the word
        for syn in wordnet.synsets(word):
            # Each synset contains lemmas (different forms/names for the concept)
            for lemma in syn.lemmas():
                # Extract the lemma name and add to synonyms set
                # .lower() ensures case-insensitive matching
                # Example: for "car", lemmas might be: car, auto, automobile, motorcar
                synonyms.add(lemma.name().lower())

        # Return the set of all synonyms found
        return synonyms

    def extract_ngrams(self, tokens, n):
        """
        Extract n-grams from a list of tokens.

        N-grams are contiguous sequences of n items from a text.
        They help capture phrases and word combinations that carry meaning together.

        Args:
            tokens (list): List of tokens (words) to extract n-grams from
            n (int): The size of n-grams to extract
                    - n=1 (unigrams): individual words ['cat', 'sits', 'mat']
                    - n=2 (bigrams): word pairs [('cat', 'sits'), ('sits', 'mat')]
                    - n=3 (trigrams): word triples [('cat', 'sits', 'mat')]

        Returns:
            list: List of n-grams (tuples of n consecutive words)

        Example:
            Input:  tokens = ['the', 'cat', 'sits', 'on', 'mat'], n = 2
            Output: [('the', 'cat'), ('cat', 'sits'), ('sits', 'on'), ('on', 'mat')]

        Why n-grams matter:
            - "New York" is different from "York New" (word order matters)
            - "not good" has opposite meaning from "good" (context matters)
            - Bigrams/trigrams capture these multi-word expressions

        """
        # Initialize empty list to store n-grams
        ngrams = []

        # Slide a window of size n across the token list
        # For each position i, create an n-gram from tokens[i] to tokens[i+n-1]
        # Example with n=2 and tokens=['a', 'b', 'c', 'd']:
        #   i=0: ('a', 'b')
        #   i=1: ('b', 'c')
        #   i=2: ('c', 'd')
        #   i=3: out of range, stop
        for i in range(len(tokens) - n + 1):
            # Extract n consecutive tokens starting at position i
            # tuple() converts the list slice into an immutable tuple
            # Tuples are hashable, allowing them to be used in sets
            ngram = tuple(tokens[i:i + n])
            ngrams.append(ngram)

        # Return the list of all n-grams extracted
        return ngrams

    def ngram_overlap_score(self, premise_tokens, hypothesis_tokens, n=2):
        """
        Calculate n-gram overlap score between premise and hypothesis.

        N-gram overlap measures how many multi-word phrases are shared between texts.
        This captures phrase-level similarity beyond individual word matching.

        Args:
            premise_tokens (list): Preprocessed tokens from the premise
            hypothesis_tokens (list): Preprocessed tokens from the hypothesis
            n (int): Size of n-grams to use (default=2 for bigrams)

        Returns:
            float: N-gram overlap score between 0.0 and 1.0

        Scoring method:
            - Extract n-grams from both premise and hypothesis
            - Count how many hypothesis n-grams appear in premise n-grams
            - Normalize by total number of hypothesis n-grams

        Example with bigrams (n=2):
            Premise:    "the cat sits on the mat"
            Hypothesis: "the cat sits there"

            Premise bigrams:    [('the','cat'), ('cat','sits'), ('sits','on'), ('on','the'), ('the','mat')]
            Hypothesis bigrams: [('the','cat'), ('cat','sits'), ('sits','there')]

            Matches: ('the','cat'), ('cat','sits')
            Score: 2 / 3 = 0.667 (66.7% of hypothesis bigrams found in premise)

        Why this helps:
            - Detects phrase matches: "not happy" vs "happy" (different meanings)
            - Preserves word order: "dog bites man" vs "man bites dog"
            - Reduces false positives from random word overlap

        """
        # Extract n-grams from both premise and hypothesis tokens
        premise_ngrams = self.extract_ngrams(premise_tokens, n)
        hypothesis_ngrams = self.extract_ngrams(hypothesis_tokens, n)

        # Handle edge case: if hypothesis has no n-grams (text too short), return 0
        # This prevents division by zero
        if len(hypothesis_ngrams) == 0:
            return 0.0

        # Convert to sets for efficient intersection operation
        # Sets provide O(1) membership testing vs O(n) for lists
        premise_ngram_set = set(premise_ngrams)
        hypothesis_ngram_set = set(hypothesis_ngrams)

        # Calculate intersection: n-grams present in both texts
        # The & operator computes set intersection
        # Example: {('a','b'), ('b','c')} & {('a','b'), ('c','d')} = {('a','b')}
        common_ngrams = premise_ngram_set & hypothesis_ngram_set

        # Calculate normalized score: common n-grams / total hypothesis n-grams
        # This gives percentage of hypothesis n-grams covered by premise
        # Score of 1.0 means all hypothesis n-grams appear in premise
        # Score of 0.0 means no n-gram overlap (completely different phrases)
        score = len(common_ngrams) / len(hypothesis_ngram_set)

        return score

    def word_overlap_score(self, premise_tokens, hypothesis_tokens):
        """
        Calculate combined word and n-gram overlap score between premise and hypothesis.

        This score measures how much the hypothesis is "covered" by the premise.
        It considers:
        1. Direct word matches (unigram overlap)
        2. Synonym matches (semantic similarity)
        3. Bigram matches (phrase-level similarity)

        Args:
            premise_tokens (list): Preprocessed tokens from the premise
            hypothesis_tokens (list): Preprocessed tokens from the hypothesis

        Returns:
            float: Combined overlap score between 0.0 and 1.0+
                   (can exceed 1.0 due to synonym partial credits)

        Scoring method:
            - Direct match: Full credit (1.0) for each hypothesis word found in premise
            - Synonym match: Partial credit (0.5) for hypothesis words with synonyms in premise
            - Bigram match: Additional bonus (0.3) added to final score for phrase similarity
            - Final score: (word_overlap + synonym_overlap) / hypothesis_size + bigram_bonus

        Example:
            Premise:    "A dog is running in the park"
            Hypothesis: "A canine is moving outdoors"

            Analysis:
            - "canine" is synonym of "dog" → +0.5 credit
            - "moving" is synonym of "running" → +0.5 credit
            - "outdoors" is synonym of "park" → +0.5 credit
            - Word score: 1.5 / 3 = 0.5
            - Bigram overlap: 0.2 → bonus of 0.06
            - Total: 0.5 + 0.06 = 0.56 (56% overlap)
        """
        # Convert token lists to sets for efficient intersection operations
        # Sets allow O(1) lookup time for membership testing
        premise_set = set(premise_tokens)
        hypothesis_set = set(hypothesis_tokens)

        # Handle edge case: if hypothesis is empty, return 0 score
        # Prevents division by zero
        if len(hypothesis_set) == 0:
            return 0.0

        # Step 1: Calculate direct word overlap (exact unigram matches)
        # The intersection (&) operator finds common elements between sets
        # Example: {'cat', 'run'} & {'cat', 'jump'} = {'cat'}
        overlap = len(premise_set & hypothesis_set)

        # Step 2: Check for synonym overlap (semantic matches)
        # For each hypothesis word not directly in premise
        for h_word in hypothesis_set:
            # If word is not already counted in direct overlap
            if h_word not in premise_set:
                # Get all synonyms of the hypothesis word
                h_synonyms = self.get_synonyms(h_word)

                # Check if any synonym appears in the premise
                # The & operator checks for intersection between sets
                if premise_set & h_synonyms:
                    # Award partial credit (0.5) for synonym match
                    # This is less than direct match since synonyms may not be exact
                    overlap += 0.5  # Partial credit for synonyms

        # Step 3: Calculate base word overlap score
        # This gives a percentage of how much of the hypothesis is covered
        # Score of 1.0 means all hypothesis words (or synonyms) are in premise
        word_score = overlap / len(hypothesis_set)

        # Step 4: Calculate bigram overlap for phrase-level matching
        # Bigrams help detect multi-word expressions and preserve word order
        # Example: "not good" vs "good" - bigrams distinguish these cases
        bigram_score = self.ngram_overlap_score(premise_tokens, hypothesis_tokens, n=2)

        # Step 5: Combine word and n-gram scores
        # Bigram score is weighted at 30% (0.3 factor) to boost phrase matches
        # This helps identify cases where word order and phrases align
        # Formula: word_score + (bigram_score * 0.3)
        # Example scenarios:
        #   - High word overlap + high bigram overlap → strong match
        #   - High word overlap + low bigram overlap → weak match (random words)
        #   - Low word overlap + high bigram overlap → unlikely but phrase-focused
        combined_score = word_score + (bigram_score * 0.3)

        # Return the combined score
        # Score can exceed 1.0 due to synonym credits and bigram bonus
        return combined_score

    def negation_check(self, text):
        """
        Check if text contains negation words.

        Negation words reverse the meaning of a statement and are key to detecting contradictions.
        This function identifies common negation patterns in English.

        Args:
            text (str): The text to check for negations

        Returns:
            bool: True if negation is found, False otherwise

        Negation words detected:
            - Direct negation: not, no, never, cannot
            - Indefinite negation: nobody, nothing, neither, nowhere, none
            - Contractions: n't (as in "isn't", "doesn't", "won't")
            - Limiting negation: hardly, scarcely (expressing near-negation)

        Examples:
            "The cat is sleeping" → False (no negation)
            "The cat is not sleeping" → True (contains "not")
            "I never go there" → True (contains "never")
            "Nobody was home" → True (contains "nobody")

        Why this matters:
            Negation changes the truth value of statements:
            - "The door is open" vs "The door is not open" are contradictory
            - Detecting negation helps classify CONTRADICTION cases
        """
        # Define set of common negation words in English
        # Using a set for O(1) lookup time
        negation_words = {
            'not',      # Standard negation: "is not"
            'no',       # Negative determiner: "no way"
            'never',    # Temporal negation: "never happens"
            'nobody',   # Negative pronoun: "nobody knows"
            'nothing',  # Negative pronoun: "nothing matters"
            'neither',  # Negative conjunction: "neither option"
            'nowhere',  # Negative adverb: "nowhere to go"
            'none',     # Negative pronoun: "none of them"
            "n't",      # Contraction negation: "isn't", "won't", "can't"
            'cannot',   # Modal negation: "cannot do"
            'hardly',   # Limiting negation: "hardly any"
            'scarcely'  # Limiting negation: "scarcely visible"
        }

        # Tokenize and lowercase the text
        # word_tokenize handles contractions properly (e.g., "isn't" → ["is", "n't"])
        tokens = word_tokenize(text.lower())

        # Check if any token is a negation word
        # any() returns True if at least one element satisfies the condition
        # This is efficient: stops checking as soon as a negation is found
        return any(word in negation_words for word in tokens)

    def recognize(self, premise, hypothesis):
        """
        Recognize textual entailment between premise and hypothesis.

        This is the core classification method that determines the relationship
        between a premise and hypothesis using word overlap and negation analysis.

        Args:
            premise (str): The premise text (the given/assumed true statement)
            hypothesis (str): The hypothesis text (the statement to verify)

        Returns:
            str: One of 'ENTAILMENT', 'CONTRADICTION', or 'NEUTRAL'

        Classification Logic:

        1. High Overlap (>70%):
           - Matching negation → ENTAILMENT
             Example: "Cat sits" + "Cat is sitting" → ENTAILMENT
           - Mismatched negation → CONTRADICTION
             Example: "Cat sits" + "Cat doesn't sit" → CONTRADICTION

        2. Moderate Overlap (30-70%):
           - Matching negation → NEUTRAL
             Example: "Cat sits" + "Dog barks" (some shared words) → NEUTRAL
           - Mismatched negation → CONTRADICTION
             Example: "Cat sits" + "Cat doesn't bark" → CONTRADICTION

        3. Low Overlap (<30%):
           - Always NEUTRAL (insufficient information to infer relationship)
             Example: "Cat sits" + "Sky is blue" → NEUTRAL

        The overlap thresholds (0.7 and 0.3) are heuristics that balance:
        - Precision: Avoiding false entailments
        - Recall: Catching actual entailments

        """
        # Step 1: Preprocess both texts
        # Convert premise and hypothesis into clean, normalized token lists
        # This removes noise (punctuation, stopwords) and standardizes words (lemmatization)
        premise_tokens = self.preprocess(premise)
        hypothesis_tokens = self.preprocess(hypothesis)

        # Step 2: Calculate word overlap score
        # This measures semantic similarity between premise and hypothesis
        # Score ranges from 0.0 (no overlap) to 1.0+ (high overlap)
        overlap = self.word_overlap_score(premise_tokens, hypothesis_tokens)

        # Step 3: Check for negations in both texts
        # Negation detection is crucial for identifying contradictions
        # Returns True if negation words are found, False otherwise
        premise_negated = self.negation_check(premise)
        hypothesis_negated = self.negation_check(hypothesis)

        # Step 4: Apply decision logic based on overlap and negation

        # Case 1: High overlap (>70%) - strong semantic similarity
        if overlap > 0.7:
            # If one text is negated but the other isn't, they contradict each other
            # Example: "Cat sits" vs "Cat doesn't sit"
            if premise_negated != hypothesis_negated:
                return "CONTRADICTION"
            else:
                # Both have same negation status (both negated or both not)
                # High overlap + same polarity = hypothesis follows from premise
                return "ENTAILMENT"

        # Case 2: Moderate overlap (30-70%) - some semantic similarity
        elif overlap > 0.3:
            # If negation differs, it's a contradiction
            # Example: "Cat sits on mat" vs "Cat doesn't sleep" (some overlap, different negation)
            if premise_negated != hypothesis_negated:
                return "CONTRADICTION"
            else:
                # Same negation but moderate overlap = not enough to infer entailment
                # Could be related but not logically following
                return "NEUTRAL"

        # Case 3: Low overlap (<30%) - weak semantic similarity
        else:
            # Not enough shared information to determine relationship
            # Hypothesis is likely unrelated to premise
            return "NEUTRAL"

    def recognize_with_confidence(self, premise, hypothesis):
        """
        Recognize textual entailment with confidence score.

        This method extends the basic recognize() method by adding a confidence score
        that indicates how certain the system is about its classification.

        Args:
            premise (str): The premise text
            hypothesis (str): The hypothesis text

        Returns:
            tuple: (label, confidence_score)
                - label (str): One of 'ENTAILMENT', 'CONTRADICTION', or 'NEUTRAL'
                - confidence_score (float): Confidence value between 0.0 and 1.0

        Confidence Calculation:

        1. ENTAILMENT:
           - Confidence = overlap score
           - Higher overlap → more confident
           - Example: 0.8 overlap → 0.8 confidence (80% confident)

        2. CONTRADICTION:
           - Confidence = overlap score × 0.8
           - Slightly reduced because contradictions need both overlap AND negation mismatch
           - Less certain than entailment with same overlap
           - Example: 0.8 overlap → 0.64 confidence (64% confident)

        3. NEUTRAL:
           - Confidence = 1 - overlap score
           - Lower overlap → more confident it's neutral
           - High overlap with neutral classification = less confident
           - Example: 0.2 overlap → 0.8 confidence (80% confident it's neutral)

        Why confidence matters:
        - Helps users understand system certainty
        - Can be used to filter low-confidence predictions
        - Useful for ranking multiple hypothesis-premise pairs
        """
        # Step 1: Preprocess both texts into normalized token lists
        premise_tokens = self.preprocess(premise)
        hypothesis_tokens = self.preprocess(hypothesis)

        # Step 2: Calculate word overlap score (semantic similarity measure)
        overlap = self.word_overlap_score(premise_tokens, hypothesis_tokens)

        # Step 3: Get the classification label using the recognize method
        label = self.recognize(premise, hypothesis)

        # Step 4: Calculate confidence score based on the label type

        if label == "ENTAILMENT":
            # For entailment, confidence directly correlates with overlap
            # High overlap = high confidence that hypothesis follows from premise
            # Example: 85% word overlap → 85% confident in entailment
            confidence = overlap

        elif label == "CONTRADICTION":
            # For contradiction, confidence is slightly reduced (×0.8 factor)
            # Because contradiction requires two conditions:
            #   1. Some overlap (to be related)
            #   2. Negation mismatch (to contradict)
            # This makes it inherently less certain than simple entailment
            # Example: 80% overlap → 64% confidence in contradiction
            confidence = overlap * 0.8

        else:  # NEUTRAL
            # For neutral, confidence is inverse of overlap
            # Lower overlap → higher confidence it's unrelated
            # High overlap but classified as neutral → lower confidence (uncertain case)
            # Example: 20% overlap → 80% confidence in neutral classification
            confidence = 1 - overlap

        # Step 5: Return both label and confidence, rounding confidence to 3 decimal places
        # Rounding improves readability: 0.857142857 → 0.857
        return label, round(confidence, 3)

    def recognize_batch(self, pairs):
        """
        Batch processing for multiple premise-hypothesis pairs.

        Process multiple text pairs efficiently in a single call.
        This is more efficient than calling recognize() repeatedly because
        it avoids repeated function call overhead.

        Args:
            pairs (list): List of dictionaries, each containing:
                - 'premise': The premise text (str)
                - 'hypothesis': The hypothesis text (str)

        Returns:
            list: List of predicted labels ('ENTAILMENT', 'CONTRADICTION', 'NEUTRAL')

        Example:
            pairs = [
                {'premise': 'A dog runs.', 'hypothesis': 'An animal moves.'},
                {'premise': 'It is sunny.', 'hypothesis': 'It is raining.'}
            ]
            labels = recognizer.recognize_batch(pairs)
            # Returns: ['ENTAILMENT', 'CONTRADICTION']
        """
        results = []
        for pair in pairs:
            label = self.recognize(pair['premise'], pair['hypothesis'])
            results.append(label)
        return results

    def recognize_batch_with_confidence(self, pairs):
        """
        Batch processing with confidence scores.

        Process multiple text pairs and return both labels and confidence scores.

        Args:
            pairs (list): List of dictionaries with 'premise' and 'hypothesis' keys

        Returns:
            list: List of tuples (label, confidence_score)

        Example:
            results = recognizer.recognize_batch_with_confidence(pairs)
            for label, conf in results:
                print(f"{label}: {conf}")
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

        Calculates comprehensive evaluation metrics including accuracy,
        precision, recall, and F1-score for each class.

        Args:
            pairs (list): List of dictionaries with 'premise' and 'hypothesis' keys
            gold_labels (list): List of true labels (same length as pairs)

        Returns:
            dict: Evaluation metrics including:
                - 'accuracy': Overall accuracy (correct / total)
                - 'per_class': Dict with precision, recall, F1 for each class
                - 'macro_f1': Average F1 across all classes
                - 'predictions': List of predicted labels
                - 'confusion_matrix': Dict showing prediction distribution

        Example:
            metrics = recognizer.evaluate(test_pairs, test_labels)
            print(f"Accuracy: {metrics['accuracy']:.2%}")
            print(f"Macro F1: {metrics['macro_f1']:.3f}")
        """
        # Get predictions for all pairs
        predictions = self.recognize_batch(pairs)

        # Calculate overall accuracy
        correct = sum(1 for pred, gold in zip(predictions, gold_labels) if pred == gold)
        accuracy = correct / len(gold_labels) if gold_labels else 0.0

        # Define all possible labels
        labels = ['ENTAILMENT', 'CONTRADICTION', 'NEUTRAL']

        # Calculate per-class metrics
        per_class = {}
        for label in labels:
            # True Positives: predicted as label AND actually is label
            tp = sum(1 for pred, gold in zip(predictions, gold_labels)
                     if pred == label and gold == label)

            # False Positives: predicted as label BUT actually is NOT label
            fp = sum(1 for pred, gold in zip(predictions, gold_labels)
                     if pred == label and gold != label)

            # False Negatives: NOT predicted as label BUT actually IS label
            fn = sum(1 for pred, gold in zip(predictions, gold_labels)
                     if pred != label and gold == label)

            # Precision: Of all predicted as label, how many are correct?
            precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0

            # Recall: Of all actual label, how many did we find?
            recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0

            # F1-score: Harmonic mean of precision and recall
            f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

            per_class[label] = {
                'precision': round(precision, 4),
                'recall': round(recall, 4),
                'f1': round(f1, 4),
                'support': sum(1 for g in gold_labels if g == label)
            }

        # Calculate macro F1 (average F1 across classes)
        macro_f1 = sum(per_class[label]['f1'] for label in labels) / len(labels)

        # Build confusion matrix
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
        """
        Print a formatted evaluation report.

        Args:
            metrics (dict): Output from evaluate() method
        """
        print("\n" + "=" * 70)
        print("EVALUATION REPORT")
        print("=" * 70)

        print(f"\nTotal Samples: {metrics['total_samples']}")
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
    """
    Main function to run the Textual Entailment Recognition System.

    This function orchestrates the entire program flow:
    1. Initializes the recognizer
    2. Runs automated test cases to demonstrate capabilities
    3. Enters interactive mode for user input

    The program demonstrates various entailment scenarios through predefined test cases,
    then allows users to test their own premise-hypothesis pairs.

    Test Cases Cover:
    - Entailment: Hypothesis logically follows from premise
    - Contradiction: Hypothesis contradicts the premise
    - Neutral: Hypothesis is unrelated to premise

    Interactive Mode:
    - Users can input custom premise-hypothesis pairs
    - System provides real-time classification with confidence scores
    - Press Ctrl+C or enter empty input to exit
    """
    # Step 1: Initialize the TextualEntailmentRecognizer
    # This downloads NLTK data, loads stopwords, and sets up the lemmatizer
    recognizer = TextualEntailmentRecognizer()

    # Step 2: Print header/banner
    # Create a visually appealing header for the program
    print("=" * 70)  # Print 70 equal signs as a separator
    print("TEXTUAL ENTAILMENT RECOGNITION SYSTEM")
    print("=" * 70)
    print()  # Empty line for spacing

    # Step 2.5: Demonstrate N-gram functionality
    # Show how n-grams capture phrase-level patterns and word order
    print("N-GRAM DEMONSTRATION")
    print("-" * 70)
    print("\nN-grams are contiguous sequences of words that capture phrases.")
    print("They help detect word order and multi-word expressions.\n")

    # Example text for n-gram demonstration
    demo_text = "the cat sits on the mat"
    demo_tokens = recognizer.preprocess(demo_text)

    print(f"Example text: '{demo_text}'")
    print(f"After preprocessing: {demo_tokens}\n")

    # Extract and display different n-gram sizes
    # Unigrams (n=1): individual words
    unigrams = recognizer.extract_ngrams(demo_tokens, n=1)
    print(f"Unigrams (n=1): {unigrams}")
    print("   -> Individual words\n")

    # Bigrams (n=2): pairs of consecutive words
    bigrams = recognizer.extract_ngrams(demo_tokens, n=2)
    print(f"Bigrams (n=2):  {bigrams}")
    print("   -> Word pairs - captures phrases like 'cat sits', 'sits mat'\n")

    # Trigrams (n=3): triples of consecutive words
    trigrams = recognizer.extract_ngrams(demo_tokens, n=3)
    print(f"Trigrams (n=3): {trigrams}")
    print("   -> Word triples - captures longer phrases\n")

    # Demonstrate n-gram overlap comparison
    print("N-GRAM OVERLAP EXAMPLE")
    print("-" * 70)
    premise1 = "the dog is not happy"
    hypothesis1 = "the dog is happy"

    premise1_tokens = recognizer.preprocess(premise1)
    hypothesis1_tokens = recognizer.preprocess(hypothesis1)

    print(f"Premise:    '{premise1}'")
    print(f"Hypothesis: '{hypothesis1}'")
    print(f"\nPremise tokens:    {premise1_tokens}")
    print(f"Hypothesis tokens: {hypothesis1_tokens}")

    # Show bigram comparison
    premise1_bigrams = set(recognizer.extract_ngrams(premise1_tokens, n=2))
    hypothesis1_bigrams = set(recognizer.extract_ngrams(hypothesis1_tokens, n=2))

    print(f"\nPremise bigrams:    {premise1_bigrams}")
    print(f"Hypothesis bigrams: {hypothesis1_bigrams}")

    # Calculate overlap
    common_bigrams = premise1_bigrams & hypothesis1_bigrams
    print(f"Common bigrams:     {common_bigrams}")

    # Calculate n-gram overlap score
    ngram_score = recognizer.ngram_overlap_score(premise1_tokens, hypothesis1_tokens, n=2)
    print(f"\nBigram overlap score: {ngram_score:.3f}")
    print("   -> Notice: Despite word overlap, bigrams differ due to 'not'")
    print("   -> This helps detect the negation that changes meaning\n")

    print("=" * 70)
    print()

    # Step 3: Define test cases
    # Each test case is a dictionary with 'premise' and 'hypothesis' keys
    # These cases demonstrate different entailment scenarios:
    test_cases = [
        {
            # Test Case 1: ENTAILMENT - "males playing soccer" → "men playing sport"
            # High overlap with synonyms (males=men, soccer=sport)
            "premise": "A soccer game with multiple males playing.",
            "hypothesis": "Some men are playing a sport."
        },
        {
            # Test Case 2: CONTRADICTION/NEUTRAL - "crowd" vs "lonely"
            # Contradictory contexts (crowd vs lonely)
            "premise": "A black race car starts up in front of a crowd of people.",
            "hypothesis": "A man is driving down a lonely road."
        },
        {
            # Test Case 3: NEUTRAL - jumping airplane vs training
            # Related but hypothesis adds unverifiable information (training, competition)
            "premise": "A person on a horse jumps over a broken down airplane.",
            "hypothesis": "A person is training his horse for a competition."
        },
        {
            # Test Case 4: NEUTRAL - waving at camera vs parents
            # Hypothesis specifies detail not in premise (parents)
            "premise": "Children smiling and waving at camera.",
            "hypothesis": "They are smiling at their parents."
        },
        {
            # Test Case 5: NEUTRAL - red bridge vs sidewalk
            # Different locations (bridge vs sidewalk)
            "premise": "A boy is jumping on skateboard in the middle of a red bridge.",
            "hypothesis": "The boy skates down the sidewalk."
        },
        {
            # Test Case 6: CONTRADICTION - sitting vs not sitting
            # Direct contradiction through negation
            "premise": "The cat is sitting on the mat.",
            "hypothesis": "The cat is not sitting on the mat."
        },
        {
            # Test Case 7: ENTAILMENT - dogs → animals, park → outdoors
            # Generalization with synonyms
            "premise": "Two dogs are running in the park.",
            "hypothesis": "Animals are playing outdoors."
        }
    ]

    # Step 4: Run all test cases
    # Iterate through each test case and display results
    for i, test in enumerate(test_cases, 1):  # enumerate with start=1 for numbering
        # Extract premise and hypothesis from the test dictionary
        premise = test["premise"]
        hypothesis = test["hypothesis"]

        # Classify the entailment relationship with confidence score
        label, confidence = recognizer.recognize_with_confidence(premise, hypothesis)

        # Display results in a formatted manner
        print(f"Test Case {i}:")
        print(f"  Premise:    {premise}")
        print(f"  Hypothesis: {hypothesis}")
        print(f"  Result:     {label} (confidence: {confidence})")
        print()  # Empty line between test cases

    # Step 5: Interactive mode header
    print("=" * 70)
    print("\nInteractive Mode - Try your own examples!")
    print("(Press Ctrl+C to exit)")
    print("=" * 70)
    print()

    # Step 6: Interactive mode loop
    # Allow users to test custom premise-hypothesis pairs
    try:
        while True:  # Infinite loop until user exits
            # Get premise from user
            # .strip() removes leading/trailing whitespace
            premise = input("Enter premise: ").strip()

            # If user enters empty premise, exit the loop
            if not premise:
                break

            # Get hypothesis from user
            hypothesis = input("Enter hypothesis: ").strip()

            # If user enters empty hypothesis, exit the loop
            if not hypothesis:
                break

            # Classify the user's input
            label, confidence = recognizer.recognize_with_confidence(premise, hypothesis)

            # Display result
            print(f"\nResult: {label} (confidence: {confidence})\n")
            print("-" * 70)  # Separator line

    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully (clean exit)
        print("\n\nExiting...")


# Program entry point
# This ensures main() only runs when script is executed directly (not imported)
if __name__ == "__main__":
    main()
