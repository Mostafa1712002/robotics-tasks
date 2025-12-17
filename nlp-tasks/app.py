"""
Flask Web Application for Textual Entailment Recognition

This provides a web interface to test the RTE system with
three different approaches:
1. Rule-Based (NLTK)
2. Embedding-Based (Sentence Transformers)
3. Transformer-Based (Pre-trained NLI model)

Usage:
    python3 app.py

Then visit: http://localhost:5000
"""

from flask import Flask, request, jsonify, render_template_string
import time

# Import recognizers
from index import TextualEntailmentRecognizer

# Lazy loading for heavy models
embedding_recognizer = None
transformer_recognizer = None

app = Flask(__name__)

# HTML Template
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Textual Entailment Recognition</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            padding: 20px;
            color: #fff;
        }
        .container {
            max-width: 900px;
            margin: 0 auto;
        }
        h1 {
            text-align: center;
            margin-bottom: 10px;
            font-size: 2.2em;
            background: linear-gradient(90deg, #00d4ff, #7b2cbf);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        .subtitle {
            text-align: center;
            color: #888;
            margin-bottom: 30px;
        }
        .card {
            background: rgba(255, 255, 255, 0.05);
            border-radius: 15px;
            padding: 25px;
            margin-bottom: 20px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.1);
        }
        .input-group {
            margin-bottom: 20px;
        }
        label {
            display: block;
            margin-bottom: 8px;
            font-weight: 500;
            color: #00d4ff;
        }
        textarea {
            width: 100%;
            padding: 15px;
            border: 2px solid rgba(255, 255, 255, 0.1);
            border-radius: 10px;
            background: rgba(0, 0, 0, 0.3);
            color: #fff;
            font-size: 16px;
            resize: vertical;
            min-height: 80px;
            transition: border-color 0.3s;
        }
        textarea:focus {
            outline: none;
            border-color: #00d4ff;
        }
        .model-select {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
            margin-bottom: 20px;
        }
        .model-option {
            flex: 1;
            min-width: 150px;
            padding: 15px;
            border: 2px solid rgba(255, 255, 255, 0.1);
            border-radius: 10px;
            background: rgba(0, 0, 0, 0.2);
            cursor: pointer;
            transition: all 0.3s;
            text-align: center;
        }
        .model-option:hover {
            border-color: #00d4ff;
        }
        .model-option.selected {
            border-color: #00d4ff;
            background: rgba(0, 212, 255, 0.1);
        }
        .model-option input {
            display: none;
        }
        .model-name {
            font-weight: bold;
            margin-bottom: 5px;
        }
        .model-desc {
            font-size: 12px;
            color: #888;
        }
        .btn {
            width: 100%;
            padding: 15px 30px;
            border: none;
            border-radius: 10px;
            background: linear-gradient(90deg, #00d4ff, #7b2cbf);
            color: #fff;
            font-size: 18px;
            font-weight: bold;
            cursor: pointer;
            transition: transform 0.2s, box-shadow 0.2s;
        }
        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 20px rgba(0, 212, 255, 0.4);
        }
        .btn:disabled {
            opacity: 0.6;
            cursor: not-allowed;
            transform: none;
        }
        .results {
            display: none;
        }
        .results.show {
            display: block;
        }
        .result-item {
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 15px;
        }
        .result-item.entailment {
            background: rgba(0, 255, 136, 0.15);
            border-left: 4px solid #00ff88;
        }
        .result-item.contradiction {
            background: rgba(255, 68, 68, 0.15);
            border-left: 4px solid #ff4444;
        }
        .result-item.neutral {
            background: rgba(255, 193, 7, 0.15);
            border-left: 4px solid #ffc107;
        }
        .result-label {
            font-size: 24px;
            font-weight: bold;
            margin-bottom: 10px;
        }
        .result-confidence {
            color: #888;
        }
        .result-time {
            font-size: 12px;
            color: #666;
            margin-top: 10px;
        }
        .examples {
            margin-top: 30px;
        }
        .example-btn {
            padding: 10px 20px;
            margin: 5px;
            border: 1px solid rgba(255, 255, 255, 0.2);
            border-radius: 20px;
            background: transparent;
            color: #888;
            cursor: pointer;
            transition: all 0.3s;
            font-size: 14px;
        }
        .example-btn:hover {
            border-color: #00d4ff;
            color: #00d4ff;
        }
        .loading {
            display: none;
            text-align: center;
            padding: 20px;
        }
        .loading.show {
            display: block;
        }
        .spinner {
            width: 40px;
            height: 40px;
            border: 4px solid rgba(255, 255, 255, 0.1);
            border-top-color: #00d4ff;
            border-radius: 50%;
            animation: spin 1s linear infinite;
            margin: 0 auto 10px;
        }
        @keyframes spin {
            to { transform: rotate(360deg); }
        }
        .info-box {
            background: rgba(0, 212, 255, 0.1);
            border: 1px solid rgba(0, 212, 255, 0.3);
            border-radius: 10px;
            padding: 15px;
            margin-bottom: 20px;
            font-size: 14px;
        }
        .info-box h3 {
            color: #00d4ff;
            margin-bottom: 10px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🧠 Textual Entailment Recognition</h1>
        <p class="subtitle">Determine if a hypothesis can be inferred from a premise</p>

        <div class="card">
            <div class="info-box">
                <h3>What is Textual Entailment?</h3>
                <p><strong>ENTAILMENT</strong>: Hypothesis is TRUE given the premise<br>
                <strong>CONTRADICTION</strong>: Hypothesis CONTRADICTS the premise<br>
                <strong>NEUTRAL</strong>: Hypothesis is UNRELATED to the premise</p>
            </div>

            <div class="input-group">
                <label for="premise">📝 Premise (the given statement)</label>
                <textarea id="premise" placeholder="Enter the premise text...">A man is playing a guitar on stage.</textarea>
            </div>

            <div class="input-group">
                <label for="hypothesis">🔍 Hypothesis (the statement to verify)</label>
                <textarea id="hypothesis" placeholder="Enter the hypothesis text...">A person is playing a musical instrument.</textarea>
            </div>

            <label>🤖 Select Model</label>
            <div class="model-select">
                <label class="model-option selected">
                    <input type="radio" name="model" value="rule_based" checked>
                    <div class="model-name">Rule-Based</div>
                    <div class="model-desc">Fast, ~40% accuracy</div>
                </label>
                <label class="model-option">
                    <input type="radio" name="model" value="embedding">
                    <div class="model-name">Embedding</div>
                    <div class="model-desc">Balanced, ~58% accuracy</div>
                </label>
                <label class="model-option">
                    <input type="radio" name="model" value="transformer">
                    <div class="model-name">Transformer</div>
                    <div class="model-desc">Best, ~88% accuracy</div>
                </label>
                <label class="model-option">
                    <input type="radio" name="model" value="all">
                    <div class="model-name">Compare All</div>
                    <div class="model-desc">Run all models</div>
                </label>
            </div>

            <button class="btn" onclick="recognize()">🚀 Analyze Entailment</button>
        </div>

        <div class="loading">
            <div class="spinner"></div>
            <p>Analyzing... (Transformer model may take a few seconds on first load)</p>
        </div>

        <div class="results card">
            <h2 style="margin-bottom: 20px;">Results</h2>
            <div id="results-container"></div>
        </div>

        <div class="examples card">
            <h3 style="margin-bottom: 15px;">📚 Try These Examples</h3>
            <button class="example-btn" onclick="loadExample(1)">Entailment: Guitar</button>
            <button class="example-btn" onclick="loadExample(2)">Contradiction: Happy/Sad</button>
            <button class="example-btn" onclick="loadExample(3)">Neutral: Walking</button>
            <button class="example-btn" onclick="loadExample(4)">Entailment: Dogs</button>
            <button class="example-btn" onclick="loadExample(5)">Contradiction: Open/Closed</button>
        </div>
    </div>

    <script>
        // Model selection
        document.querySelectorAll('.model-option').forEach(option => {
            option.addEventListener('click', function() {
                document.querySelectorAll('.model-option').forEach(o => o.classList.remove('selected'));
                this.classList.add('selected');
                this.querySelector('input').checked = true;
            });
        });

        // Example data
        const examples = {
            1: {
                premise: "A man is playing a guitar on stage.",
                hypothesis: "A person is playing a musical instrument."
            },
            2: {
                premise: "The boy is happy and smiling.",
                hypothesis: "The boy is sad and crying."
            },
            3: {
                premise: "A woman is walking down the street.",
                hypothesis: "She is going to buy groceries."
            },
            4: {
                premise: "Two dogs are running in the park.",
                hypothesis: "Animals are playing outdoors."
            },
            5: {
                premise: "The store is closed for the night.",
                hypothesis: "The store is open for business."
            }
        };

        function loadExample(num) {
            document.getElementById('premise').value = examples[num].premise;
            document.getElementById('hypothesis').value = examples[num].hypothesis;
        }

        async function recognize() {
            const premise = document.getElementById('premise').value.trim();
            const hypothesis = document.getElementById('hypothesis').value.trim();
            const model = document.querySelector('input[name="model"]:checked').value;

            if (!premise || !hypothesis) {
                alert('Please enter both premise and hypothesis');
                return;
            }

            // Show loading
            document.querySelector('.loading').classList.add('show');
            document.querySelector('.results').classList.remove('show');
            document.querySelector('.btn').disabled = true;

            try {
                const response = await fetch('/api/recognize', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({premise, hypothesis, model})
                });

                const data = await response.json();
                displayResults(data);
            } catch (error) {
                alert('Error: ' + error.message);
            } finally {
                document.querySelector('.loading').classList.remove('show');
                document.querySelector('.btn').disabled = false;
            }
        }

        function displayResults(data) {
            const container = document.getElementById('results-container');
            container.innerHTML = '';

            if (data.results) {
                data.results.forEach(result => {
                    const div = document.createElement('div');
                    div.className = `result-item ${result.label.toLowerCase()}`;
                    div.innerHTML = `
                        <div style="font-size: 12px; color: #888; margin-bottom: 5px;">${result.model}</div>
                        <div class="result-label">${getLabelEmoji(result.label)} ${result.label}</div>
                        <div class="result-confidence">Confidence: ${(result.confidence * 100).toFixed(1)}%</div>
                        <div class="result-time">Time: ${result.time.toFixed(3)}s</div>
                    `;
                    container.appendChild(div);
                });
            }

            document.querySelector('.results').classList.add('show');
        }

        function getLabelEmoji(label) {
            switch(label) {
                case 'ENTAILMENT': return '✅';
                case 'CONTRADICTION': return '❌';
                case 'NEUTRAL': return '➖';
                default: return '❓';
            }
        }
    </script>
</body>
</html>
'''

# Initialize rule-based recognizer (lightweight)
rule_based_recognizer = TextualEntailmentRecognizer()


def get_embedding_recognizer():
    """Lazy load embedding recognizer."""
    global embedding_recognizer
    if embedding_recognizer is None:
        from embedding_recognizer import EmbeddingEntailmentRecognizer
        embedding_recognizer = EmbeddingEntailmentRecognizer()
    return embedding_recognizer


def get_transformer_recognizer():
    """Lazy load transformer recognizer."""
    global transformer_recognizer
    if transformer_recognizer is None:
        from transformer_recognizer import TransformerEntailmentRecognizer
        transformer_recognizer = TransformerEntailmentRecognizer()
    return transformer_recognizer


@app.route('/')
def home():
    """Serve the main page."""
    return render_template_string(HTML_TEMPLATE)


@app.route('/api/recognize', methods=['POST'])
def recognize():
    """API endpoint for entailment recognition."""
    data = request.json
    premise = data.get('premise', '')
    hypothesis = data.get('hypothesis', '')
    model = data.get('model', 'rule_based')

    results = []

    if model in ['rule_based', 'all']:
        start = time.time()
        label, confidence = rule_based_recognizer.recognize_with_confidence(premise, hypothesis)
        elapsed = time.time() - start
        results.append({
            'model': 'Rule-Based (NLTK)',
            'label': label,
            'confidence': float(confidence),
            'time': float(elapsed)
        })

    if model in ['embedding', 'all']:
        try:
            recognizer = get_embedding_recognizer()
            start = time.time()
            label, confidence = recognizer.recognize_with_confidence(premise, hypothesis)
            elapsed = time.time() - start
            results.append({
                'model': 'Embedding (Sentence-Transformers)',
                'label': label,
                'confidence': float(confidence),
                'time': float(elapsed)
            })
        except Exception as e:
            results.append({
                'model': 'Embedding (Sentence-Transformers)',
                'label': 'ERROR',
                'confidence': 0,
                'time': 0,
                'error': str(e)
            })

    if model in ['transformer', 'all']:
        try:
            recognizer = get_transformer_recognizer()
            start = time.time()
            label, confidence = recognizer.recognize_with_confidence(premise, hypothesis)
            elapsed = time.time() - start
            results.append({
                'model': 'Transformer (DeBERTa-NLI)',
                'label': label,
                'confidence': float(confidence),
                'time': float(elapsed)
            })
        except Exception as e:
            results.append({
                'model': 'Transformer (DeBERTa-NLI)',
                'label': 'ERROR',
                'confidence': 0,
                'time': 0,
                'error': str(e)
            })

    return jsonify({'results': results})


@app.route('/api/health')
def health():
    """Health check endpoint."""
    return jsonify({'status': 'ok', 'models': ['rule_based', 'embedding', 'transformer']})


if __name__ == '__main__':
    print("Starting Textual Entailment Recognition Server...")
    print("Visit: http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False)
