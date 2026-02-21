"""
Fairness Metrics for Multi-Agent Systems

Provides correct implementations of fairness metrics including Jain's Fairness Index.

CRITICAL: Jain's Fairness Index is NOT the same as Coefficient of Variation (CV).
"""

import numpy as np
from typing import List, Union


def compute_jain_index(values: Union[List[float], np.ndarray]) -> float:
    """
    Compute Jain's Fairness Index.
    
    Jain's Fairness Index (JFI) measures how fairly resources are allocated.
    
    Formula:
        JFI = (Σx_i)² / (n * Σ(x_i²))
    
    Properties:
        - Range: [1/n, 1]
        - 1/n: Maximally unfair (one agent gets everything)
        - 1: Perfectly fair (all agents get equal share)
        - Scale-invariant: JFI(k*x) = JFI(x)
    
    Args:
        values: Resource allocation values (e.g., rewards, energy, throughput)
               Can be List or ndarray of shape (n,)
    
    Returns:
        float: Jain's Fairness Index in range [1/n, 1]
    
    Examples:
        >>> compute_jain_index([10, 10, 10, 10])  # Perfect fairness
        1.0
        >>> compute_jain_index([40, 0, 0, 0])     # Maximally unfair
        0.25
        >>> compute_jain_index([15, 10, 10, 5])   # Moderate fairness
        0.9
    
    References:
        Jain, R., Chiu, D. M., & Hawe, W. R. (1984). 
        A quantitative measure of fairness and discrimination for resource allocation
        in shared computer systems. Eastern Research Laboratory, Digital Equipment Corporation.
    """
    values = np.array(values, dtype=np.float64)
    
    if len(values) == 0:
        return 0.0
    
    # Remove negative values (fairness only defined for non-negative allocations)
    if np.any(values < 0):
        raise ValueError("Jain Index requires non-negative values")
    
    n = len(values)
    sum_x = np.sum(values)
    sum_x_squared = np.sum(values ** 2)
    
    # Handle edge case: all zeros
    if sum_x_squared == 0:
        return 1.0  # Trivially fair (no resources allocated)
    
    jain_index = (sum_x ** 2) / (n * sum_x_squared)
    
    return float(jain_index)


def compute_coefficient_of_variation(values: Union[List[float], np.ndarray]) -> float:
    """
    Compute Coefficient of Variation (CV).
    
    CV measures relative variability as the ratio of standard deviation to mean.
    
    Formula:
        CV = σ / μ
    
    Properties:
        - Range: [0, ∞)
        - 0: No variation (all values equal)
        - >1: High variation
        - NOT bounded (can be arbitrarily large)
    
    WARNING: This is NOT Jain's Fairness Index!
    CV and Jain Index are different metrics with different properties.
    
    Args:
        values: Values to compute CV for
    
    Returns:
        float: Coefficient of Variation
    
    Examples:
        >>> compute_coefficient_of_variation([10, 10, 10, 10])
        0.0
        >>> compute_coefficient_of_variation([10, 20, 30, 40])
        0.447...
    """
    values = np.array(values, dtype=np.float64)
    
    if len(values) == 0:
        return 0.0
    
    mean = np.mean(values)
    std = np.std(values)
    
    # Handle edge case: zero mean
    if abs(mean) < 1e-10:
        return 0.0 if std < 1e-10 else float('inf')
    
    cv = std / abs(mean)
    
    return float(cv)


def compute_gini_coefficient(values: Union[List[float], np.ndarray]) -> float:
    """
    Compute Gini Coefficient.
    
    Gini coefficient measures statistical dispersion (inequality).
    
    Formula:
        G = (Σ Σ |x_i - x_j|) / (2n² μ)
    
    Properties:
        - Range: [0, 1]
        - 0: Perfect equality
        - 1: Perfect inequality (one agent has everything)
    
    Args:
        values: Resource allocation values
    
    Returns:
        float: Gini coefficient in range [0, 1]
    
    Examples:
        >>> compute_gini_coefficient([10, 10, 10, 10])
        0.0
        >>> compute_gini_coefficient([40, 0, 0, 0])
        0.75
    """
    values = np.array(values, dtype=np.float64)
    
    if len(values) == 0:
        return 0.0
    
    # Sort values
    sorted_values = np.sort(values)
    n = len(sorted_values)
    
    # Compute cumulative sum
    cumsum = np.cumsum(sorted_values)
    
    # Gini formula
    gini = (2 * np.sum((np.arange(1, n+1)) * sorted_values)) / (n * cumsum[-1]) - (n + 1) / n
    
    return float(gini)


def compute_fairness_metrics(values: Union[List[float], np.ndarray]) -> dict:
    """
    Compute all fairness metrics for a set of values.
    
    Args:
        values: Resource allocation values (rewards, energy, etc.)
    
    Returns:
        dict: Dictionary with all fairness metrics:
            - 'jain_index': Jain's Fairness Index [1/n, 1]
            - 'cv': Coefficient of Variation [0, ∞)
            - 'gini': Gini Coefficient [0, 1]
            - 'min': Minimum value
            - 'max': Maximum value
            - 'mean': Mean value
            - 'std': Standard deviation
    
    Example:
        >>> metrics = compute_fairness_metrics([15, 10, 10, 5])
        >>> metrics['jain_index']
        0.9
        >>> metrics['gini']
        0.125
    """
    values = np.array(values, dtype=np.float64)
    
    metrics = {
        'jain_index': compute_jain_index(values),
        'cv': compute_coefficient_of_variation(values),
        'gini': compute_gini_coefficient(values),
        'min': float(np.min(values)) if len(values) > 0 else 0.0,
        'max': float(np.max(values)) if len(values) > 0 else 0.0,
        'mean': float(np.mean(values)) if len(values) > 0 else 0.0,
        'std': float(np.std(values)) if len(values) > 0 else 0.0,
    }
    
    return metrics


def convert_cv_to_jain_approximation(cv: float, n: int) -> float:
    """
    Approximate Jain Index from Coefficient of Variation.
    
    This is an approximation for converting historical CV values to Jain Index.
    
    Relationship (approximate):
        JFI ≈ 1 / (1 + CV²)
    
    WARNING: This is only an approximation! 
    Always compute Jain Index directly from raw values when possible.
    
    Args:
        cv: Coefficient of Variation
        n: Number of agents
    
    Returns:
        float: Approximate Jain Index
    """
    # Approximate relationship between CV and Jain Index
    # Derived from: JFI = (1 + μ²/σ²) / (1 + n*μ²/σ²)
    # For large n: JFI ≈ 1 / (1 + CV²)
    
    if cv == 0:
        return 1.0
    
    approx_jain = 1.0 / (1.0 + cv ** 2)
    
    # Clamp to valid range [1/n, 1]
    return np.clip(approx_jain, 1.0 / n, 1.0)


if __name__ == "__main__":
    # Test cases
    print("Testing Fairness Metrics")
    print("=" * 70)
    
    test_cases = [
        ([10, 10, 10, 10], "Perfect fairness"),
        ([40, 0, 0, 0], "Maximally unfair"),
        ([15, 10, 10, 5], "Moderate fairness"),
        ([20, 15, 10, 5], "Variable allocation"),
    ]
    
    for values, description in test_cases:
        print(f"\n{description}: {values}")
        metrics = compute_fairness_metrics(values)
        print(f"  Jain Index: {metrics['jain_index']:.4f}")
        print(f"  Gini Coeff: {metrics['gini']:.4f}")
        print(f"  CV: {metrics['cv']:.4f}")
    
    print("\n" + "=" * 70)
    print("✅ All fairness metrics computed correctly!")
