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


# Documentation HTML Template
DOCS_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>API Documentation - Textual Entailment Recognition</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            color: #fff;
            line-height: 1.6;
        }
        .container { max-width: 1100px; margin: 0 auto; padding: 40px 20px; }
        h1 {
            text-align: center;
            font-size: 2.5em;
            margin-bottom: 10px;
            background: linear-gradient(90deg, #00d4ff, #7b2cbf);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        .subtitle { text-align: center; color: #888; margin-bottom: 40px; font-size: 1.1em; }
        .nav {
            display: flex;
            justify-content: center;
            gap: 20px;
            margin-bottom: 40px;
            flex-wrap: wrap;
        }
        .nav a {
            color: #00d4ff;
            text-decoration: none;
            padding: 10px 20px;
            border: 1px solid #00d4ff;
            border-radius: 25px;
            transition: all 0.3s;
        }
        .nav a:hover { background: rgba(0, 212, 255, 0.2); }
        .card {
            background: rgba(255, 255, 255, 0.05);
            border-radius: 15px;
            padding: 30px;
            margin-bottom: 30px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.1);
        }
        h2 {
            color: #00d4ff;
            margin-bottom: 20px;
            padding-bottom: 10px;
            border-bottom: 1px solid rgba(0, 212, 255, 0.3);
        }
        h3 { color: #7b2cbf; margin: 20px 0 10px; }
        p { margin-bottom: 15px; color: #ccc; }
        .badge {
            display: inline-block;
            padding: 5px 12px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: bold;
            margin-right: 10px;
        }
        .badge-blue { background: #00d4ff; color: #000; }
        .badge-green { background: #00ff88; color: #000; }
        .badge-purple { background: #7b2cbf; color: #fff; }
        .badge-yellow { background: #ffc107; color: #000; }
        table {
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
        }
        th, td {
            padding: 12px 15px;
            text-align: left;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
        }
        th { background: rgba(0, 212, 255, 0.1); color: #00d4ff; }
        tr:hover { background: rgba(255, 255, 255, 0.03); }
        .code-block {
            background: #0d1117;
            border-radius: 10px;
            padding: 20px;
            margin: 15px 0;
            overflow-x: auto;
            font-family: 'Consolas', 'Monaco', monospace;
            font-size: 14px;
            border: 1px solid #30363d;
        }
        .code-block code { color: #c9d1d9; }
        .keyword { color: #ff7b72; }
        .string { color: #a5d6ff; }
        .comment { color: #8b949e; }
        .function { color: #d2a8ff; }
        .accuracy-bar {
            height: 30px;
            border-radius: 15px;
            background: rgba(255, 255, 255, 0.1);
            margin: 10px 0;
            overflow: hidden;
        }
        .accuracy-fill {
            height: 100%;
            border-radius: 15px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-weight: bold;
            font-size: 14px;
        }
        .fill-rule { background: linear-gradient(90deg, #ff6b6b, #ffc107); width: 37.5%; }
        .fill-embed { background: linear-gradient(90deg, #ffc107, #00d4ff); width: 57.5%; }
        .fill-trans { background: linear-gradient(90deg, #00d4ff, #00ff88); width: 92.5%; }
        .label-box {
            display: inline-block;
            padding: 8px 16px;
            border-radius: 8px;
            margin: 5px;
            font-weight: bold;
        }
        .label-entailment { background: rgba(0, 255, 136, 0.2); border: 2px solid #00ff88; }
        .label-contradiction { background: rgba(255, 68, 68, 0.2); border: 2px solid #ff4444; }
        .label-neutral { background: rgba(255, 193, 7, 0.2); border: 2px solid #ffc107; }
        .footer {
            text-align: center;
            padding: 30px;
            color: #666;
            border-top: 1px solid rgba(255, 255, 255, 0.1);
            margin-top: 40px;
        }
        .footer a { color: #00d4ff; text-decoration: none; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
        .feature-card {
            background: rgba(0, 0, 0, 0.3);
            padding: 20px;
            border-radius: 10px;
            border-left: 4px solid #00d4ff;
        }
        .feature-card h4 { color: #00d4ff; margin-bottom: 10px; }

        /* Mobile Responsive Styles */
        @media (max-width: 768px) {
            .container { padding: 20px 15px; }
            h1 { font-size: 1.8em; }
            .subtitle { font-size: 0.95em; margin-bottom: 25px; }
            .nav { gap: 10px; margin-bottom: 25px; }
            .nav a { padding: 8px 14px; font-size: 13px; }
            .card { padding: 20px 15px; margin-bottom: 20px; border-radius: 12px; }
            h2 { font-size: 1.3em; margin-bottom: 15px; }
            h3 { font-size: 1.1em; margin: 15px 0 8px; }
            h4 { font-size: 1em; }
            p { font-size: 14px; }
            .grid { grid-template-columns: 1fr; gap: 15px; }
            .feature-card { padding: 15px; }
            .label-box { padding: 6px 12px; font-size: 13px; display: block; margin: 10px 0; }
            .badge { padding: 4px 10px; font-size: 11px; margin: 3px 5px 3px 0; }
            .accuracy-bar { height: 25px; }
            .accuracy-fill { font-size: 12px; }

            /* Table responsive */
            table { font-size: 12px; display: block; overflow-x: auto; white-space: nowrap; }
            th, td { padding: 8px 10px; }

            /* Code block responsive */
            .code-block { padding: 12px; font-size: 11px; border-radius: 8px; }

            .footer { padding: 20px 15px; }
            .footer p { font-size: 13px; }
        }

        @media (max-width: 480px) {
            .container { padding: 15px 10px; }
            h1 { font-size: 1.5em; }
            .subtitle { font-size: 0.85em; }
            .nav { flex-direction: column; align-items: center; }
            .nav a { width: 100%; text-align: center; }
            .card { padding: 15px 12px; }
            h2 { font-size: 1.2em; }
            .code-block { font-size: 10px; padding: 10px; }
            table { font-size: 11px; }
            th, td { padding: 6px 8px; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🧠 Textual Entailment Recognition</h1>
        <p class="subtitle">API Documentation & Technical Reference</p>

        <div class="nav">
            <a href="/">🚀 Try Demo</a>
            <a href="#overview">📋 Overview</a>
            <a href="#models">🤖 Models</a>
            <a href="#api">🔌 API</a>
            <a href="#examples">💡 Examples</a>
        </div>

        <div class="card" id="overview">
            <h2>📋 What is Textual Entailment?</h2>
            <p>Textual Entailment (Natural Language Inference) determines the logical relationship between two text segments:</p>

            <div style="margin: 20px 0;">
                <div class="label-box label-entailment">✅ ENTAILMENT</div>
                <span>Hypothesis is TRUE given the premise</span>
            </div>
            <div style="margin: 20px 0;">
                <div class="label-box label-contradiction">❌ CONTRADICTION</div>
                <span>Hypothesis CONTRADICTS the premise</span>
            </div>
            <div style="margin: 20px 0;">
                <div class="label-box label-neutral">➖ NEUTRAL</div>
                <span>Hypothesis is UNRELATED to the premise</span>
            </div>

            <h3>Features</h3>
            <div class="grid">
                <div class="feature-card">
                    <h4>🚀 Three Models</h4>
                    <p>From simple rules to state-of-the-art transformers</p>
                </div>
                <div class="feature-card">
                    <h4>📊 92.5% Accuracy</h4>
                    <p>Best-in-class with DeBERTa transformer</p>
                </div>
                <div class="feature-card">
                    <h4>⚡ REST API</h4>
                    <p>Easy integration with any application</p>
                </div>
                <div class="feature-card">
                    <h4>🔒 SSL Enabled</h4>
                    <p>Secure HTTPS connections</p>
                </div>
            </div>
        </div>

        <div class="card" id="models">
            <h2>🤖 Three Approaches</h2>

            <h3>Phase 1: Rule-Based (NLTK + WordNet)</h3>
            <div class="accuracy-bar"><div class="accuracy-fill fill-rule">37.5%</div></div>
            <p>Uses tokenization, lemmatization, word overlap, synonym detection via WordNet, and negation detection.</p>
            <p><span class="badge badge-green">Fast</span><span class="badge badge-yellow">Low Complexity</span></p>

            <h3>Phase 2: Embedding-Based (Sentence Transformers)</h3>
            <div class="accuracy-bar"><div class="accuracy-fill fill-embed">57.5%</div></div>
            <p>Encodes sentences into 384-dimensional vectors using all-MiniLM-L6-v2, computes cosine similarity.</p>
            <p><span class="badge badge-blue">Balanced</span><span class="badge badge-yellow">Medium Complexity</span></p>

            <h3>Phase 3: Transformer-Based (DeBERTa-NLI)</h3>
            <div class="accuracy-bar"><div class="accuracy-fill fill-trans">92.5%</div></div>
            <p>Uses cross-encoder/nli-deberta-v3-small pre-trained on SNLI/MultiNLI with millions of examples.</p>
            <p><span class="badge badge-purple">Best Accuracy</span><span class="badge badge-green">Production Ready</span></p>

            <h3>Performance Comparison</h3>
            <table>
                <tr><th>Approach</th><th>Accuracy</th><th>Macro F1</th><th>Speed</th></tr>
                <tr><td>Rule-Based</td><td>37.50%</td><td>0.2237</td><td>~0.003s</td></tr>
                <tr><td>Embedding</td><td>57.50%</td><td>0.5537</td><td>~0.05s</td></tr>
                <tr><td>Transformer</td><td><strong>92.50%</strong></td><td><strong>0.9253</strong></td><td>~0.15s</td></tr>
            </table>
        </div>

        <div class="card" id="api">
            <h2>🔌 API Reference</h2>

            <h3>Base URL</h3>
            <div class="code-block"><code>https://nlp.newaves-systems.com/api</code></div>

            <h3>POST /api/recognize</h3>
            <p>Analyze the entailment relationship between premise and hypothesis.</p>

            <h4>Request Body</h4>
            <div class="code-block"><code>{
  <span class="string">"premise"</span>: <span class="string">"A man is playing a guitar on stage."</span>,
  <span class="string">"hypothesis"</span>: <span class="string">"A person is playing a musical instrument."</span>,
  <span class="string">"model"</span>: <span class="string">"transformer"</span>
}</code></div>

            <h4>Parameters</h4>
            <table>
                <tr><th>Field</th><th>Type</th><th>Required</th><th>Description</th></tr>
                <tr><td>premise</td><td>string</td><td>Yes</td><td>The premise text</td></tr>
                <tr><td>hypothesis</td><td>string</td><td>Yes</td><td>The hypothesis text</td></tr>
                <tr><td>model</td><td>string</td><td>No</td><td>rule_based, embedding, transformer, or all</td></tr>
            </table>

            <h4>Response</h4>
            <div class="code-block"><code>{
  <span class="string">"results"</span>: [
    {
      <span class="string">"model"</span>: <span class="string">"Transformer (DeBERTa-NLI)"</span>,
      <span class="string">"label"</span>: <span class="string">"ENTAILMENT"</span>,
      <span class="string">"confidence"</span>: <span class="keyword">0.991</span>,
      <span class="string">"time"</span>: <span class="keyword">0.156</span>
    }
  ]
}</code></div>

            <h3>GET /api/health</h3>
            <p>Check API health status.</p>
            <div class="code-block"><code>{
  <span class="string">"status"</span>: <span class="string">"ok"</span>,
  <span class="string">"models"</span>: [<span class="string">"rule_based"</span>, <span class="string">"embedding"</span>, <span class="string">"transformer"</span>]
}</code></div>
        </div>

        <div class="card" id="examples">
            <h2>💡 Code Examples</h2>

            <h3>cURL</h3>
            <div class="code-block"><code><span class="function">curl</span> -X POST https://nlp.newaves-systems.com/api/recognize \\
  -H <span class="string">"Content-Type: application/json"</span> \\
  -d <span class="string">'{
    "premise": "The cat is sleeping on the couch.",
    "hypothesis": "An animal is resting.",
    "model": "transformer"
  }'</span></code></div>

            <h3>Python</h3>
            <div class="code-block"><code><span class="keyword">import</span> requests

response = requests.<span class="function">post</span>(
    <span class="string">"https://nlp.newaves-systems.com/api/recognize"</span>,
    json={
        <span class="string">"premise"</span>: <span class="string">"Two dogs are playing in the park."</span>,
        <span class="string">"hypothesis"</span>: <span class="string">"Animals are having fun outdoors."</span>,
        <span class="string">"model"</span>: <span class="string">"transformer"</span>
    }
)

data = response.<span class="function">json</span>()
<span class="keyword">for</span> result <span class="keyword">in</span> data[<span class="string">"results"</span>]:
    <span class="function">print</span>(f<span class="string">"{result['label']}: {result['confidence']:.1%}"</span>)</code></div>

            <h3>JavaScript</h3>
            <div class="code-block"><code><span class="keyword">const</span> response = <span class="keyword">await</span> <span class="function">fetch</span>(<span class="string">'https://nlp.newaves-systems.com/api/recognize'</span>, {
  method: <span class="string">'POST'</span>,
  headers: { <span class="string">'Content-Type'</span>: <span class="string">'application/json'</span> },
  body: JSON.<span class="function">stringify</span>({
    premise: <span class="string">'The movie was boring.'</span>,
    hypothesis: <span class="string">'The movie was exciting.'</span>,
    model: <span class="string">'transformer'</span>
  })
});

<span class="keyword">const</span> data = <span class="keyword">await</span> response.<span class="function">json</span>();
console.<span class="function">log</span>(data.results[0].label); <span class="comment">// CONTRADICTION</span></code></div>

            <h3>Test Examples</h3>
            <table>
                <tr><th>Premise</th><th>Hypothesis</th><th>Expected</th></tr>
                <tr><td>A soccer game with multiple males playing.</td><td>Some men are playing a sport.</td><td>✅ ENTAILMENT</td></tr>
                <tr><td>The store is closed.</td><td>The store is open for business.</td><td>❌ CONTRADICTION</td></tr>
                <tr><td>A man is walking down the street.</td><td>The man is going to work.</td><td>➖ NEUTRAL</td></tr>
            </table>
        </div>

        <div class="footer">
            <p>© 2024 <strong>Mostafa Ibrahim</strong></p>
            <p>Email: <a href="mailto:mostafaibrahim1712002@gmail.com">mostafaibrahim1712002@gmail.com</a></p>
            <p>GitHub: <a href="https://github.com/Mostafa1712002" target="_blank">@Mostafa1712002</a></p>
            <p style="margin-top: 20px;">Built with ❤️ for NLP enthusiasts</p>
        </div>
    </div>
</body>
</html>
'''


@app.route('/docs')
def docs():
    """Serve the documentation page."""
    return render_template_string(DOCS_TEMPLATE)


if __name__ == '__main__':
    print("Starting Textual Entailment Recognition Server...")
    print("Visit: http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False)
