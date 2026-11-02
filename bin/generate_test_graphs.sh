. .venv/bin/activate

{
    python test_scripts/analyze_accuracy.py
    python test_scripts/analyze_kidnapping.py
    python test_scripts/analyze_particles.py
    python test_scripts/analyze_recovery.py
    python test_scripts/analyze_runtime.py
} > analysis_results.txt 2>&1