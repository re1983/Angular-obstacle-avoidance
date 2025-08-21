# Angular-obstacle-avoidance

##run

python BearingRateGraph_cleaned.py

You can change setting in run_simulation()

## Documentation

### Paper-Style Overviews
- [Angle-Only Collision Avoidance via CBDR](./Angle-Only-Collision-Avoidance.md) - System overview and methodology
- [Lyapunov-Barrier-Analysis.md](./Lyapunov-Barrier-Analysis.md) - UUB rationale and barrier analysis

### Mathematical Analysis
- **[Complete Mathematical Proof](./lyapunov_analysis/mathematical_proof.md)** - Formal Lyapunov stability proofs with LaTeX equations
- **[Analysis System Documentation](./lyapunov_analysis/README.md)** - Comprehensive user guide and API reference

## Lyapunov Stability Analysis

### Comprehensive Mathematical Analysis
A complete Lyapunov stability analysis system is available in the **`lyapunov_analysis/`** folder, providing:

- **Mathematical Proofs**: [Complete formal proof with LaTeX equations](./lyapunov_analysis/mathematical_proof.md)
- **Numerical Verification**: Monte Carlo testing with randomized scenarios (>95% success rate)  
- **Parameter Sensitivity**: Analysis of optimal R_safe and α_nav values
- **Visualization**: Comprehensive plots of stability properties and system behavior

#### Key Documentation
- 📖 **[Mathematical Proof](./lyapunov_analysis/mathematical_proof.md)** - Formal Lyapunov stability proofs
- 📋 **[System README](./lyapunov_analysis/README.md)** - Detailed usage instructions and API reference

#### Quick Start
```bash
cd lyapunov_analysis
python main_analysis_runner.py
```

#### Results Include
- Safety guarantees (collision avoidance)
- Boundedness proofs (system stability)  
- Ultimate boundedness verification (convergence)
- Optimal parameter recommendations
- Statistical performance analysis

See **[lyapunov_analysis/README.md](./lyapunov_analysis/README.md)** for detailed usage instructions and **[mathematical_proof.md](./lyapunov_analysis/mathematical_proof.md)** for complete mathematical foundations.

## Verification

Run a quick boundedness/UUB safety check for both controllers:

```bash
python3 debug_tests/verify_uub.py
```

This prints min distance, max angular size, and CBDR-threshold hit ratio; adjust `R_SAFE` or initial conditions inside the script to explore sensitivity.

### Advanced Verification
For comprehensive stability analysis with mathematical proofs and statistical validation:
```bash
cd lyapunov_analysis
python main_analysis_runner.py
```
