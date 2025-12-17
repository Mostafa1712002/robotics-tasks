# 🧠 Textual Entailment Recognition System

<div align="center">

![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)
![Flask](https://img.shields.io/badge/Flask-3.1-green.svg)
![PyTorch](https://img.shields.io/badge/PyTorch-2.9-red.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

**A state-of-the-art Natural Language Inference (NLI) system that determines logical relationships between text pairs.**

[Live Demo](https://nlp.newaves-systems.com) | [API Documentation](#-api-reference) | [Installation](#-installation)

</div>

---

## 📋 Table of Contents

- [What is Textual Entailment?](#-what-is-textual-entailment)
- [Features](#-features)
- [Live Demo](#-live-demo)
- [Three Approaches](#-three-approaches)
- [Performance Comparison](#-performance-comparison)
- [Installation](#-installation)
- [Usage](#-usage)
- [API Reference](#-api-reference)
- [Project Structure](#-project-structure)
- [Technical Details](#-technical-details)
- [Examples](#-examples)
- [Future Enhancements](#-future-enhancements)
- [License](#-license)
- [Author](#-author)

---

## 🎯 What is Textual Entailment?

Textual Entailment (also known as Natural Language Inference - NLI) is the task of determining the logical relationship between two text segments:

| Label | Description | Example |
|-------|-------------|---------|
| **ENTAILMENT** | Hypothesis is TRUE given the premise | Premise: "A dog is running" → Hypothesis: "An animal is moving" ✅ |
| **CONTRADICTION** | Hypothesis CONTRADICTS the premise | Premise: "The store is closed" → Hypothesis: "The store is open" ❌ |
| **NEUTRAL** | Hypothesis is UNRELATED to the premise | Premise: "A man walks" → Hypothesis: "He is going to work" ➖ |

---

## ✨ Features

- 🚀 **Three Recognition Approaches** - From simple rules to state-of-the-art transformers
- 🌐 **Beautiful Web Interface** - Modern dark theme with real-time results
- 📊 **Confidence Scores** - Know how certain the model is about its prediction
- ⚡ **REST API** - Easy integration with any application
- 📈 **92.5% Accuracy** - State-of-the-art performance with transformer model
- 🔒 **SSL Enabled** - Secure HTTPS connections
- 📱 **Responsive Design** - Works on desktop and mobile

---

## 🌐 Live Demo

**🔗 [https://nlp.newaves-systems.com](https://nlp.newaves-systems.com)**

Try the system live with:
- Interactive web interface
- All three models available
- Pre-loaded example test cases
- Real-time confidence scores

---

## 🔬 Three Approaches

### Phase 1: Rule-Based (NLTK + WordNet)

```
Accuracy: 37.50% | Speed: Fast | Complexity: Low
```

**How it works:**
- Tokenization and lemmatization with NLTK
- Word overlap calculation between premise and hypothesis
- Synonym detection using WordNet lexical database
- Negation detection for contradiction signals
- N-gram analysis for phrase-level matching

**Best for:** Quick prototyping, educational purposes, understanding NLI basics

---

### Phase 2: Embedding-Based (Sentence Transformers)

```
Accuracy: 57.50% | Speed: Medium | Complexity: Medium
```

**How it works:**
- Encodes sentences into 384-dimensional vectors using `all-MiniLM-L6-v2`
- Computes cosine similarity between premise and hypothesis embeddings
- Detects contradiction signals (negation, antonyms)
- Combines semantic similarity with rule-based features

**Best for:** Balanced accuracy/speed trade-off, resource-constrained environments

---

### Phase 3: Transformer-Based (DeBERTa-NLI)

```
Accuracy: 92.50% | Speed: Slower | Complexity: High
```

**How it works:**
- Uses `cross-encoder/nli-deberta-v3-small` pre-trained on SNLI/MultiNLI
- Cross-attention between premise and hypothesis tokens
- Fine-tuned on millions of human-annotated NLI examples
- Softmax over three classes for probability distribution

**Best for:** Production use, high-accuracy requirements, critical applications

---

## 📊 Performance Comparison

### Accuracy & F1 Scores

| Approach | Accuracy | Macro F1 | Entailment F1 | Contradiction F1 | Neutral F1 |
|----------|----------|----------|---------------|------------------|------------|
| Rule-Based | 37.50% | 0.2237 | 0.0000 | 0.1429 | 0.5283 |
| Embedding | 57.50% | 0.5537 | 0.5588 | 0.4242 | 0.6780 |
| **Transformer** | **92.50%** | **0.9253** | **0.9200** | **0.9630** | **0.8929** |

### Improvement Over Baseline

| Metric | Embedding vs Rule-Based | Transformer vs Rule-Based |
|--------|------------------------|---------------------------|
| Accuracy | +20.00% | **+55.00%** |
| Macro F1 | +0.3300 | **+0.7016** |

### Speed Comparison

| Approach | Model Load Time | Inference Time (per sample) |
|----------|-----------------|----------------------------|
| Rule-Based | ~4s (NLTK data) | ~0.003s |
| Embedding | ~3s | ~0.05s |
| Transformer | ~10s | ~0.15s |

---

## 🛠 Installation

### Prerequisites

- Python 3.10+
- pip package manager
- 2GB+ RAM (for transformer model)

### Local Installation

```bash
# Clone the repository
git clone https://github.com/Mostafa1712002/robotics-tasks.git
cd robotics-tasks/nlp-tasks

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# or: venv\Scripts\activate  # Windows

# Install dependencies
pip install nltk sentence-transformers transformers torch flask

# Download NLTK data (automatic on first run)
python -c "import nltk; nltk.download('punkt'); nltk.download('stopwords'); nltk.download('wordnet')"
```

### Server Deployment

```bash
# Install on Ubuntu/Debian server
pip3 install nltk sentence-transformers transformers flask gunicorn

# For CPU-only PyTorch (smaller, faster install)
pip3 install torch --index-url https://download.pytorch.org/whl/cpu

# Run with gunicorn
gunicorn -w 4 -b 0.0.0.0:5000 app:app
```

---

## 📖 Usage

### Web Interface

1. Visit [https://nlp.newaves-systems.com](https://nlp.newaves-systems.com)
2. Enter your **Premise** (the given statement)
3. Enter your **Hypothesis** (the statement to verify)
4. Select a model or "Compare All"
5. Click **"Analyze Entailment"**

### Python API

```python
# Rule-Based
from index import TextualEntailmentRecognizer

recognizer = TextualEntailmentRecognizer()
label, confidence = recognizer.recognize_with_confidence(
    premise="A dog is running in the park.",
    hypothesis="An animal is outdoors."
)
print(f"{label}: {confidence}")  # ENTAILMENT: 0.85

# Embedding-Based
from embedding_recognizer import EmbeddingEntailmentRecognizer

recognizer = EmbeddingEntailmentRecognizer()
label, confidence = recognizer.recognize_with_confidence(
    premise="A man is playing guitar.",
    hypothesis="Someone is making music."
)
print(f"{label}: {confidence}")  # ENTAILMENT: 0.78

# Transformer-Based (Best Accuracy)
from transformer_recognizer import TransformerEntailmentRecognizer

recognizer = TransformerEntailmentRecognizer()
label, confidence = recognizer.recognize_with_confidence(
    premise="The restaurant is closed.",
    hypothesis="The restaurant is open for dinner."
)
print(f"{label}: {confidence}")  # CONTRADICTION: 0.99
```

### Batch Processing

```python
pairs = [
    {"premise": "Dogs are animals.", "hypothesis": "Dogs are living beings."},
    {"premise": "It is sunny.", "hypothesis": "It is raining."},
    {"premise": "She is reading.", "hypothesis": "She likes books."}
]

# Process multiple pairs
results = recognizer.recognize_batch(pairs)
# ['ENTAILMENT', 'CONTRADICTION', 'NEUTRAL']

# With confidence scores
results = recognizer.recognize_batch_with_confidence(pairs)
# [('ENTAILMENT', 0.95), ('CONTRADICTION', 0.88), ('NEUTRAL', 0.72)]
```

### Evaluation

```python
from index import TextualEntailmentRecognizer
from evaluate_snli import create_mini_dataset

# Load test data
pairs, labels = create_mini_dataset()

# Evaluate
recognizer = TextualEntailmentRecognizer()
metrics = recognizer.evaluate(pairs, labels)

# Print report
recognizer.print_evaluation_report(metrics)
```

---

## 🔌 API Reference

### Base URL

```
https://nlp.newaves-systems.com/api
```

### Endpoints

#### `POST /api/recognize`

Analyze the entailment relationship between premise and hypothesis.

**Request Body:**

```json
{
  "premise": "A man is playing a guitar on stage.",
  "hypothesis": "A person is playing a musical instrument.",
  "model": "transformer"
}
```

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `premise` | string | Yes | The premise text |
| `hypothesis` | string | Yes | The hypothesis text |
| `model` | string | No | Model to use: `rule_based`, `embedding`, `transformer`, or `all` (default: `rule_based`) |

**Response:**

```json
{
  "results": [
    {
      "model": "Transformer (DeBERTa-NLI)",
      "label": "ENTAILMENT",
      "confidence": 0.991,
      "time": 0.156
    }
  ]
}
```

#### `GET /api/health`

Check API health status.

**Response:**

```json
{
  "status": "ok",
  "models": ["rule_based", "embedding", "transformer"]
}
```

### Example API Calls

**cURL:**

```bash
curl -X POST https://nlp.newaves-systems.com/api/recognize \
  -H "Content-Type: application/json" \
  -d '{
    "premise": "The cat is sleeping on the couch.",
    "hypothesis": "An animal is resting.",
    "model": "all"
  }'
```

**Python requests:**

```python
import requests

response = requests.post(
    "https://nlp.newaves-systems.com/api/recognize",
    json={
        "premise": "Two dogs are playing in the park.",
        "hypothesis": "Animals are having fun outdoors.",
        "model": "transformer"
    }
)

data = response.json()
for result in data["results"]:
    print(f"{result['model']}: {result['label']} ({result['confidence']:.1%})")
```

**JavaScript fetch:**

```javascript
const response = await fetch('https://nlp.newaves-systems.com/api/recognize', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    premise: 'The movie was boring.',
    hypothesis: 'The movie was exciting.',
    model: 'transformer'
  })
});

const data = await response.json();
console.log(data.results[0].label); // CONTRADICTION
```

---

## 📁 Project Structure

```
nlp-tasks/
├── app.py                      # Flask web application
├── index.py                    # Rule-based recognizer (Phase 1)
├── embedding_recognizer.py     # Embedding-based recognizer (Phase 2)
├── transformer_recognizer.py   # Transformer-based recognizer (Phase 3)
├── evaluate_snli.py            # SNLI evaluation script
├── compare_all.py              # Compare all three approaches
├── compare_approaches.py       # Compare rule-based vs embedding
├── test_ngrams.py              # N-gram functionality tests
└── README.md                   # This documentation
```

### File Descriptions

| File | Lines | Description |
|------|-------|-------------|
| `app.py` | 522 | Flask web server with HTML/CSS/JS UI |
| `index.py` | 809 | TextualEntailmentRecognizer class with NLTK |
| `embedding_recognizer.py` | 583 | EmbeddingEntailmentRecognizer with Sentence Transformers |
| `transformer_recognizer.py` | 328 | TransformerEntailmentRecognizer with DeBERTa |
| `evaluate_snli.py` | 399 | SNLI dataset evaluation and benchmarking |
| `compare_all.py` | 150 | Full comparison of all three approaches |

---

## 🔧 Technical Details

### Models Used

| Phase | Model | Size | Source |
|-------|-------|------|--------|
| 1 | NLTK + WordNet | ~50MB | nltk.org |
| 2 | all-MiniLM-L6-v2 | ~90MB | HuggingFace |
| 3 | nli-deberta-v3-small | ~180MB | HuggingFace |

### System Requirements

| Requirement | Minimum | Recommended |
|-------------|---------|-------------|
| Python | 3.10 | 3.11+ |
| RAM | 2GB | 4GB+ |
| Storage | 500MB | 1GB |
| CPU | Any | Multi-core |
| GPU | Not required | Optional (faster inference) |

### Dependencies

```
nltk>=3.8
sentence-transformers>=2.2
transformers>=4.30
torch>=2.0
flask>=3.0
numpy>=1.24
scipy>=1.10
scikit-learn>=1.3
```

---

## 💡 Examples

### Entailment Examples

| Premise | Hypothesis | Result |
|---------|------------|--------|
| A soccer game with multiple males playing. | Some men are playing a sport. | ✅ ENTAILMENT |
| Two dogs are running in the park. | Animals are playing outdoors. | ✅ ENTAILMENT |
| A chef is preparing a meal. | Someone is cooking. | ✅ ENTAILMENT |
| The musician strums a guitar. | Someone is making music. | ✅ ENTAILMENT |

### Contradiction Examples

| Premise | Hypothesis | Result |
|---------|------------|--------|
| The cat is sitting on the mat. | The cat is not sitting on the mat. | ❌ CONTRADICTION |
| The store is closed. | The store is open for business. | ❌ CONTRADICTION |
| The boy is happy. | The boy is sad and upset. | ❌ CONTRADICTION |
| It is raining outside. | The weather is sunny and clear. | ❌ CONTRADICTION |

### Neutral Examples

| Premise | Hypothesis | Result |
|---------|------------|--------|
| A man is walking down the street. | The man is going to work. | ➖ NEUTRAL |
| A woman is reading a book. | The book is interesting. | ➖ NEUTRAL |
| Children are playing in the yard. | They are playing soccer. | ➖ NEUTRAL |
| A dog is running in the field. | The dog is chasing a rabbit. | ➖ NEUTRAL |

---

## 🚀 Future Enhancements

- [ ] Multi-language support (Arabic, French, Spanish)
- [ ] Fine-tuning on domain-specific data
- [ ] Larger transformer models (DeBERTa-v3-base, RoBERTa-large)
- [ ] Explainability features (attention visualization)
- [ ] Batch API endpoint for multiple pairs
- [ ] WebSocket support for real-time streaming
- [ ] Docker containerization
- [ ] Kubernetes deployment manifests

---

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👤 Author

**Mostafa Ibrahim**

- Email: mostafaibrahim1712002@gmail.com
- GitHub: [@Mostafa1712002](https://github.com/Mostafa1712002)

---

## 🙏 Acknowledgments

- [NLTK](https://www.nltk.org/) - Natural Language Toolkit
- [Sentence Transformers](https://www.sbert.net/) - Sentence embeddings
- [HuggingFace](https://huggingface.co/) - Transformer models
- [SNLI Dataset](https://nlp.stanford.edu/projects/snli/) - Stanford NLI corpus
- [DeBERTa](https://github.com/microsoft/DeBERTa) - Microsoft's DeBERTa model

---

<div align="center">

**⭐ Star this repository if you found it helpful!**

Made with ❤️ for NLP enthusiasts

</div>
