# Textual Entailment Recognition System

A Natural Language Processing (NLP) system that recognizes textual entailment relationships between premise and hypothesis text pairs using NLTK.

## What is Textual Entailment?

Textual entailment is the task of determining whether a hypothesis can be inferred from a given premise. The system classifies text pairs into three categories:

- **ENTAILMENT**: The hypothesis is true given the premise
- **CONTRADICTION**: The hypothesis contradicts the premise
- **NEUTRAL**: The hypothesis is unrelated to the premise

## Features

- **Word Overlap Analysis**: Measures semantic similarity between premise and hypothesis
- **Synonym Detection**: Uses WordNet to identify semantically related words
- **Negation Detection**: Identifies contradictions through negation words
- **Text Preprocessing**: Tokenization, lemmatization, and stopword removal
- **Confidence Scoring**: Provides confidence levels for predictions
- **Interactive Mode**: Test your own premise-hypothesis pairs

## Installation

### Prerequisites

- Python 3.x
- pip package manager

### Install Dependencies

```bash
python -m pip install nltk
```

The required NLTK data packages (punkt, stopwords, wordnet) will be downloaded automatically on first run.

## Usage

### Run the Program

```bash
python index.py
```

### Program Flow

1. **Automated Test Cases**: The program runs 7 pre-configured test cases demonstrating various entailment scenarios
2. **Interactive Mode**: After the test cases, you can input your own premise-hypothesis pairs

### Example Usage

```
Enter premise: A cat is sleeping on the couch.
Enter hypothesis: An animal is resting on furniture.

Result: ENTAILMENT (confidence: 0.75)
```

## How It Works

### Architecture

The `TextualEntailmentRecognizer` class uses the following approach:

1. **Preprocessing**:
   - Tokenize text into words
   - Convert to lowercase
   - Remove punctuation and stopwords
   - Lemmatize words to their base forms

2. **Feature Extraction**:
   - Calculate word overlap between premise and hypothesis
   - Detect synonyms using WordNet
   - Check for negation words

3. **Classification Logic**:
   - High overlap (>70%) with matching negation → ENTAILMENT
   - High overlap with mismatched negation → CONTRADICTION
   - Moderate overlap (30-70%) → NEUTRAL or CONTRADICTION (based on negation)
   - Low overlap (<30%) → NEUTRAL

4. **Confidence Scoring**:
   - Based on word overlap percentage
   - Adjusted for classification type

## Test Cases

The system includes 7 diverse test cases:

| Premise | Hypothesis | Expected Result |
|---------|-----------|-----------------|
| A soccer game with multiple males playing. | Some men are playing a sport. | ENTAILMENT |
| A black race car starts up in front of a crowd of people. | A man is driving down a lonely road. | CONTRADICTION |
| A person on a horse jumps over a broken down airplane. | A person is training his horse for a competition. | NEUTRAL |
| Children smiling and waving at camera. | They are smiling at their parents. | NEUTRAL |
| A boy is jumping on skateboard in the middle of a red bridge. | The boy skates down the sidewalk. | NEUTRAL |
| The cat is sitting on the mat. | The cat is not sitting on the mat. | CONTRADICTION |
| Two dogs are running in the park. | Animals are playing outdoors. | ENTAILMENT |

## API Reference

### Class: `TextualEntailmentRecognizer`

#### Methods

##### `__init__()`
Initialize the recognizer and download required NLTK data.

##### `recognize(premise: str, hypothesis: str) -> str`
Classify the entailment relationship.

**Parameters**:
- `premise`: The premise text
- `hypothesis`: The hypothesis text

**Returns**: One of `"ENTAILMENT"`, `"CONTRADICTION"`, or `"NEUTRAL"`

##### `recognize_with_confidence(premise: str, hypothesis: str) -> tuple`
Classify with confidence score.

**Parameters**:
- `premise`: The premise text
- `hypothesis`: The hypothesis text

**Returns**: Tuple of `(label, confidence_score)`

**Example**:
```python
recognizer = TextualEntailmentRecognizer()
label, confidence = recognizer.recognize_with_confidence(
    "A dog is running in the park.",
    "An animal is outdoors."
)
print(f"{label} (confidence: {confidence})")
# Output: ENTAILMENT (confidence: 0.85)
```

## Limitations

- **Simple heuristics**: Uses rule-based methods rather than machine learning
- **Limited semantic understanding**: May miss complex relationships
- **Language-specific**: Designed for English text only
- **WordNet dependency**: Synonym detection limited to WordNet coverage

## Future Enhancements

- Implement deep learning models (BERT, RoBERTa) for better accuracy
- Add support for multiple languages
- Incorporate named entity recognition
- Use sentence embeddings for semantic similarity
- Train on standard datasets (SNLI, MultiNLI)
- Add batch processing capability

## References

- [NLTK Documentation](https://www.nltk.org/)
- [WordNet](https://wordnet.princeton.edu/)
- [Textual Entailment (Wikipedia)](https://en.wikipedia.org/wiki/Textual_entailment)
- [SNLI Dataset](https://nlp.stanford.edu/projects/snli/)
- [MultiNLI Dataset](https://cims.nyu.edu/~sbowman/multinli/)

## License

MIT License - Feel free to use and modify this code for your projects.

## Author

Developed by [mostafa1712002]. Contact: [mostafaibrahim1712002@gmail.com]

## Contributing

Contributions are welcome! Feel free to:
- Report bugs
- Suggest new features
- Submit pull requests
- Improve documentation

---

**Note**: This is an educational implementation demonstrating NLP concepts. For production use, consider using pre-trained transformer models like BERT or RoBERTa fine-tuned on entailment datasets.
