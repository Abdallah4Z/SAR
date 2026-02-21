"""
Statistical tests for comparing FL-MARL algorithm combinations.

Provides t-tests, Mann-Whitney U tests, confidence intervals,
and effect size measures (Cohen's d) to determine whether performance
differences between algorithms are statistically significant.

Usage:
    from src.evaluation.statistical_tests import (
        paired_t_test, bootstrap_ci, cohens_d, compare_distributions
    )
"""

import numpy as np
from typing import Dict, List, Optional, Tuple
from scipy import stats


def paired_t_test(
    scores_a: List[float],
    scores_b: List[float],
    alpha: float = 0.05
) -> Dict:
    """
    Paired t-test to compare two algorithms on the same evaluation episodes.

    Use this when both algorithms are evaluated on the same environment
    seeds/episodes, so per-episode differences are meaningful.

    Args:
        scores_a: Performance scores for algorithm A (list of episode rewards)
        scores_b: Performance scores for algorithm B
        alpha:    Significance level (default 0.05)

    Returns:
        dict: {
            't_statistic': float,
            'p_value': float,
            'significant': bool,
            'mean_diff': float,
            'ci_low': float,    # 95% CI for mean difference
            'ci_high': float,
        }

    Example:
        >>> result = paired_t_test([1.0, 1.2, 0.9], [0.8, 1.0, 0.7])
        >>> print(result['significant'])
        True
    """
    a = np.array(scores_a, dtype=np.float64)
    b = np.array(scores_b, dtype=np.float64)
    assert len(a) == len(b), "Paired t-test requires equal-length arrays"

    t_stat, p_value = stats.ttest_rel(a, b)
    diff = a - b
    mean_diff = float(np.mean(diff))
    se = float(stats.sem(diff))
    df = len(diff) - 1
    t_critical = stats.t.ppf(1 - alpha / 2, df)
    ci_low = mean_diff - t_critical * se
    ci_high = mean_diff + t_critical * se

    return {
        't_statistic': float(t_stat),
        'p_value': float(p_value),
        'significant': bool(p_value < alpha),
        'mean_diff': mean_diff,
        'ci_low': float(ci_low),
        'ci_high': float(ci_high),
        'n_samples': len(a),
        'alpha': alpha,
    }


def independent_t_test(
    scores_a: List[float],
    scores_b: List[float],
    alpha: float = 0.05,
    equal_var: bool = False
) -> Dict:
    """
    Welch's t-test for independent samples (Welch's if equal_var=False).

    Use this when comparing algorithms evaluated on different seeds.

    Args:
        scores_a:  Performance scores for algorithm A
        scores_b:  Performance scores for algorithm B
        alpha:     Significance level
        equal_var: If True, assume equal variances (Student's t-test)

    Returns:
        dict: Statistical test results
    """
    a = np.array(scores_a, dtype=np.float64)
    b = np.array(scores_b, dtype=np.float64)

    t_stat, p_value = stats.ttest_ind(a, b, equal_var=equal_var)
    mean_diff = float(np.mean(a) - np.mean(b))

    return {
        't_statistic': float(t_stat),
        'p_value': float(p_value),
        'significant': bool(p_value < alpha),
        'mean_a': float(np.mean(a)),
        'mean_b': float(np.mean(b)),
        'mean_diff': mean_diff,
        'std_a': float(np.std(a)),
        'std_b': float(np.std(b)),
        'n_a': len(a),
        'n_b': len(b),
        'alpha': alpha,
    }


def mann_whitney_u_test(
    scores_a: List[float],
    scores_b: List[float],
    alpha: float = 0.05
) -> Dict:
    """
    Mann-Whitney U test (non-parametric alternative to independent t-test).

    Use when normality cannot be assumed (e.g., small samples or skewed
    reward distributions).

    Args:
        scores_a: Performance scores for algorithm A
        scores_b: Performance scores for algorithm B
        alpha:    Significance level

    Returns:
        dict: Statistical test results
    """
    a = np.array(scores_a, dtype=np.float64)
    b = np.array(scores_b, dtype=np.float64)

    u_stat, p_value = stats.mannwhitneyu(a, b, alternative='two-sided')

    return {
        'u_statistic': float(u_stat),
        'p_value': float(p_value),
        'significant': bool(p_value < alpha),
        'median_a': float(np.median(a)),
        'median_b': float(np.median(b)),
        'n_a': len(a),
        'n_b': len(b),
        'alpha': alpha,
    }


def cohens_d(scores_a: List[float], scores_b: List[float]) -> float:
    """
    Compute Cohen's d effect size between two groups.

    Interpretation:
        |d| < 0.2  -> negligible
        |d| < 0.5  -> small
        |d| < 0.8  -> medium
        |d| >= 0.8 -> large

    Args:
        scores_a: Scores for group A
        scores_b: Scores for group B

    Returns:
        float: Cohen's d (positive means A > B)

    Example:
        >>> d = cohens_d([1.0, 1.1, 0.9], [0.5, 0.6, 0.4])
        >>> print(f"Effect size: {d:.2f}")
    """
    a = np.array(scores_a, dtype=np.float64)
    b = np.array(scores_b, dtype=np.float64)

    mean_diff = np.mean(a) - np.mean(b)
    # Pooled standard deviation
    pooled_std = np.sqrt(
        ((len(a) - 1) * np.var(a, ddof=1) + (len(b) - 1) * np.var(b, ddof=1))
        / (len(a) + len(b) - 2)
    )
    if pooled_std == 0:
        return 0.0
    return float(mean_diff / pooled_std)


def bootstrap_ci(
    scores: List[float],
    confidence: float = 0.95,
    n_bootstrap: int = 1000,
    statistic=np.mean,
    seed: int = 42
) -> Tuple[float, float]:
    """
    Bootstrap confidence interval for a statistic.

    More robust than parametric CI when distribution shape is unknown.

    Args:
        scores:     Observed scores
        confidence: Confidence level (default 0.95 for 95% CI)
        n_bootstrap: Number of bootstrap resamples
        statistic:  Function to compute (default np.mean)
        seed:       Random seed for reproducibility

    Returns:
        Tuple[float, float]: (ci_low, ci_high)

    Example:
        >>> ci = bootstrap_ci([1.0, 0.9, 1.1, 0.8, 1.2])
        >>> print(f"95% CI: [{ci[0]:.3f}, {ci[1]:.3f}]")
    """
    rng = np.random.default_rng(seed)
    data = np.array(scores, dtype=np.float64)
    bootstrap_stats = []
    for _ in range(n_bootstrap):
        sample = rng.choice(data, size=len(data), replace=True)
        bootstrap_stats.append(statistic(sample))
    alpha = 1 - confidence
    ci_low = float(np.percentile(bootstrap_stats, 100 * alpha / 2))
    ci_high = float(np.percentile(bootstrap_stats, 100 * (1 - alpha / 2)))
    return ci_low, ci_high


def compare_distributions(
    scores_a: List[float],
    scores_b: List[float],
    alpha: float = 0.05,
    label_a: str = "A",
    label_b: str = "B"
) -> Dict:
    """
    Comprehensive comparison between two score distributions.

    Runs both t-test and Mann-Whitney, computes effect size and
    bootstrap CI for the difference in means.

    Args:
        scores_a: Scores for algorithm A
        scores_b: Scores for algorithm B
        alpha:    Significance level
        label_a:  Name of algorithm A (for output)
        label_b:  Name of algorithm B (for output)

    Returns:
        dict: Full comparison report with all statistics
    """
    a = np.array(scores_a, dtype=np.float64)
    b = np.array(scores_b, dtype=np.float64)

    t_result = independent_t_test(scores_a, scores_b, alpha=alpha)
    mw_result = mann_whitney_u_test(scores_a, scores_b, alpha=alpha)
    d = cohens_d(scores_a, scores_b)

    # Bootstrap CI for the difference in means
    # CRITICAL FIX: Use seeded RNG for reproducibility
    rng = np.random.default_rng(42)
    diffs = (rng.choice(a, size=(1000, len(a)), replace=True).mean(axis=1)
             - rng.choice(b, size=(1000, len(b)), replace=True).mean(axis=1))
    ci_low_diff = float(np.percentile(diffs, 2.5))
    ci_high_diff = float(np.percentile(diffs, 97.5))

    # Effect size label
    abs_d = abs(d)
    if abs_d < 0.2:
        effect_label = "negligible"
    elif abs_d < 0.5:
        effect_label = "small"
    elif abs_d < 0.8:
        effect_label = "medium"
    else:
        effect_label = "large"

    winner = label_a if np.mean(a) > np.mean(b) else label_b

    return {
        'label_a': label_a,
        'label_b': label_b,
        'mean_a': float(np.mean(a)),
        'mean_b': float(np.mean(b)),
        'std_a': float(np.std(a, ddof=1)),
        'std_b': float(np.std(b, ddof=1)),
        'median_a': float(np.median(a)),
        'median_b': float(np.median(b)),
        't_test': t_result,
        'mann_whitney': mw_result,
        'cohens_d': d,
        'effect_size_label': effect_label,
        'ci_diff_low': ci_low_diff,
        'ci_diff_high': ci_high_diff,
        'significant_t': t_result['significant'],
        'significant_mw': mw_result['significant'],
        'winner': winner,
        'n_a': len(a),
        'n_b': len(b),
    }


if __name__ == '__main__':
    """Smoke test for statistical tests."""
    import numpy as np
    np.random.seed(0)

    print("Testing statistical tests...\n")

    a = np.random.normal(1.0, 0.1, 30).tolist()
    b = np.random.normal(0.8, 0.15, 30).tolist()

    print("Test 1: Paired t-test")
    r = paired_t_test(a[:20], b[:20])
    print(f"  t={r['t_statistic']:.3f}, p={r['p_value']:.4f}, "
          f"significant={r['significant']}, mean_diff={r['mean_diff']:.4f}")
    assert 't_statistic' in r and 'p_value' in r
    print("  \u2713 Paired t-test works")

    print("\nTest 2: Independent t-test")
    r = independent_t_test(a, b)
    print(f"  t={r['t_statistic']:.3f}, p={r['p_value']:.4f}, significant={r['significant']}")
    assert r['mean_a'] > r['mean_b']
    print("  \u2713 Independent t-test works")

    print("\nTest 3: Mann-Whitney U")
    r = mann_whitney_u_test(a, b)
    print(f"  U={r['u_statistic']:.1f}, p={r['p_value']:.4f}, significant={r['significant']}")
    print("  \u2713 Mann-Whitney works")

    print("\nTest 4: Cohen's d")
    d = cohens_d(a, b)
    print(f"  d={d:.3f}")
    assert d > 0
    print("  \u2713 Cohen's d works")

    print("\nTest 5: Bootstrap CI")
    ci = bootstrap_ci(a)
    print(f"  95% CI: [{ci[0]:.3f}, {ci[1]:.3f}]")
    assert ci[0] < np.mean(a) < ci[1]
    print("  \u2713 Bootstrap CI works")

    print("\nTest 6: Full comparison")
    report = compare_distributions(a, b, label_a="FedAdam+MAPPO", label_b="FedAvg+MAPPO")
    print(f"  Winner: {report['winner']}")
    print(f"  Effect: {report['cohens_d']:.2f} ({report['effect_size_label']})")
    print(f"  Diff CI: [{report['ci_diff_low']:.3f}, {report['ci_diff_high']:.3f}]")
    assert 'winner' in report and 'effect_size_label' in report
    print("  \u2713 Full comparison works")

    print("\n\u2705 All statistical tests passed!")
