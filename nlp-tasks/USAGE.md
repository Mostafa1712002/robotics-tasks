# How to Test N-gram Functionality

This guide shows you how to test and use the n-gram features in the Textual Entailment Recognition System.

## Quick Start

### Method 1: Run the Complete Test Suite

```bash
python test_ngrams.py
```

This will run all n-gram tests including:
- N-gram extraction (unigrams, bigrams, trigrams)
- N-gram overlap scoring
- Entailment detection with n-grams
- Phrase detection examples

### Method 2: Run the Main Program

```bash
python index.py
```

This will:
1. Show an n-gram demonstration
2. Run 7 test cases
3. Enter interactive mode where you can test your own examples

### Method 3: Use in Python Code

Create your own test script:

```python
from index import TextualEntailmentRecognizer

# Initialize the recognizer
recognizer = TextualEntailmentRecognizer()

# Test 1: Extract n-grams from text
text = "The cat sits on the mat"
tokens = recognizer.preprocess(text)

# Get bigrams
bigrams = recognizer.extract_ngrams(tokens, n=2)
print(f"Bigrams: {bigrams}")

# Get trigrams
trigrams = recognizer.extract_ngrams(tokens, n=3)
print(f"Trigrams: {trigrams}")

# Test 2: Calculate n-gram overlap
premise = "I like pizza"
hypothesis = "I love pizza"

premise_tokens = recognizer.preprocess(premise)
hypothesis_tokens = recognizer.preprocess(hypothesis)

# Calculate bigram overlap
overlap = recognizer.ngram_overlap_score(premise_tokens, hypothesis_tokens, n=2)
print(f"Bigram overlap: {overlap:.3f}")

# Test 3: Full entailment detection (uses n-grams internally)
label, confidence = recognizer.recognize_with_confidence(premise, hypothesis)
print(f"Result: {label} (confidence: {confidence})")
```

## Understanding N-gram Output

### 1. N-gram Extraction

**Input:** "The cat sits on the mat"

**After preprocessing:** ['cat', 'sits', 'mat']
(stopwords 'the', 'on' removed)

**Unigrams (n=1):**
```
[('cat',), ('sits',), ('mat',)]
```
→ Individual words

**Bigrams (n=2):**
```
[('cat', 'sits'), ('sits', 'mat')]
```
→ Consecutive word pairs

**Trigrams (n=3):**
```
[('cat', 'sits', 'mat')]
```
→ Consecutive word triples

### 2. N-gram Overlap Scores

Scores range from 0.0 to 1.0:
- **1.0** = Perfect overlap (all hypothesis n-grams in premise)
- **0.5** = Half overlap
- **0.0** = No overlap

**Example:**

```
Premise:    "The dog bites the man"
Hypothesis: "The man bites the dog"

Unigram overlap: 1.000  (same words: dog, bites, man)
Bigram overlap:  0.000  (different word order!)
```

This shows how n-grams detect word order differences!

### 3. Entailment Detection

The system combines:
- **Word overlap** (unigrams)
- **Synonym matching** (WordNet)
- **Bigram overlap** (phrases)

**Example:**

```python
premise = "A dog is running in the park"
hypothesis = "An animal is running in the park"

# Result: ENTAILMENT (confidence: 0.817)
# - Word overlap: high (running, park)
# - Synonyms: dog → animal
# - Bigram overlap: 0.500 (some phrases match)
```

## Common Use Cases

### Case 1: Test Individual Sentences

```python
from index import TextualEntailmentRecognizer

recognizer = TextualEntailmentRecognizer()

# Your text
text = "I do not like this movie"
tokens = recognizer.preprocess(text)
bigrams = recognizer.extract_ngrams(tokens, n=2)

print(f"Original: {text}")
print(f"Tokens: {tokens}")
print(f"Bigrams: {bigrams}")
```

### Case 2: Compare Two Sentences

```python
premise = "The cat is sleeping"
hypothesis = "The cat is not sleeping"

premise_tokens = recognizer.preprocess(premise)
hypothesis_tokens = recognizer.preprocess(hypothesis)

# Check overlap
bigram_score = recognizer.ngram_overlap_score(premise_tokens, hypothesis_tokens, n=2)
print(f"Bigram overlap: {bigram_score:.3f}")

# Get entailment prediction
label, conf = recognizer.recognize_with_confidence(premise, hypothesis)
print(f"Prediction: {label} (confidence: {conf})")
```

### Case 3: Test Multiple Sentences

```python
sentences = [
    "The weather is sunny today",
    "I enjoy playing basketball",
    "Machine learning is fascinating",
    "New York City is beautiful"
]

for sentence in sentences:
    tokens = recognizer.preprocess(sentence)
    bigrams = recognizer.extract_ngrams(tokens, n=2)
    print(f"{sentence}")
    print(f"  Bigrams: {bigrams}\n")
```

## What N-grams Help Detect

### 1. Word Order
```
"dog bites man" ≠ "man bites dog"
→ Same words, different bigrams!
```

### 2. Negation
```
"I like pizza" vs "I do not like pizza"
→ Bigrams: ('like', 'pizza') vs ('like', 'pizza')
   (Note: 'not' is removed as stopword, but negation_check() detects it)
```

### 3. Phrases
```
"New York City"
→ Bigrams: ('new', 'york'), ('york', 'city')
   Keeps "New York" as connected phrase
```

### 4. Multi-word Expressions
```
"Machine learning"
→ Bigram: ('machine', 'learning')
   Treats as single concept
```

## Interpreting Test Results

When you run `python test_ngrams.py`, you'll see:

### ✓ **Test 1: N-gram Extraction**
Shows how text is broken into n-grams of different sizes

### ✓ **Test 2: N-gram Overlap Scoring**
Compares different text pairs and shows overlap scores

**Key insight:** Same words but different order = 0% bigram overlap!

### ✓ **Test 3: Entailment Detection**
Shows how n-grams improve entailment classification

**Key insight:** Bigram scores help distinguish similar vs identical phrases

### ✓ **Test 4: Phrase Detection**
Demonstrates how bigrams capture multi-word expressions

**Key insight:** "New York" stays together as a phrase unit

## Troubleshooting

### Issue: Unicode Error
**Solution:** Already fixed! We use ASCII arrows (`->`) instead of Unicode (`→`)

### Issue: Empty N-grams
**Cause:** Text too short after preprocessing
**Solution:** Use longer sentences or check stopword removal

### Issue: Low Overlap Scores
**Cause:** Normal! Different sentences should have low overlap
**What to check:** Compare the actual tokens and n-grams to understand why

## Files in This Project

- **index.py** - Main program with n-gram implementation
- **test_ngrams.py** - Comprehensive test suite
- **USAGE.md** - This usage guide
- **README.md** - Full project documentation

## Next Steps

1. **Run tests:** `python test_ngrams.py`
2. **Try interactive mode:** `python index.py` (skip to interactive section)
3. **Experiment:** Modify test_ngrams.py with your own examples
4. **Learn more:** Read README.md for full documentation

## References

- [N-gram on Wikipedia](https://en.wikipedia.org/wiki/N-gram)
- [NLTK Documentation](https://www.nltk.org/)
- [Textual Entailment](https://en.wikipedia.org/wiki/Textual_entailment)

---

**Happy Testing! 🚀**
