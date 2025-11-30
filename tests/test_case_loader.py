#!/usr/bin/env python3
"""Utility for loading test cases from YAML configuration."""

import yaml
from pathlib import Path
from typing import Dict, List, Any


def get_test_cases_path() -> Path:
    """Get the path to test_cases.yaml."""
    return Path(__file__).parent / "test_cases.yaml"


def load_test_cases() -> Dict[str, List[Dict[str, Any]]]:
    """
    Load test cases from YAML file.

    Returns:
        Dictionary with test case categories as keys and lists of test cases as values.
        Each test case contains 'name', 'description', 'params', and 'expected' fields.
    """
    yaml_path = get_test_cases_path()

    if not yaml_path.exists():
        raise FileNotFoundError(f"Test cases file not found: {yaml_path}")

    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            if data is None:
                raise ValueError(f"YAML file is empty or invalid: {yaml_path}")
            return data
    except yaml.YAMLError as e:
        raise ValueError(f"Failed to parse YAML file {yaml_path}: {e}")
    except Exception as e:
        raise ValueError(f"Failed to load test cases from {yaml_path}: {e}")


def get_test_category(category: str) -> List[Dict[str, Any]]:
    """
    Get test cases from a specific category.

    Args:
        category: Category name (e.g., 'overspeed_tests', 'error_tests')

    Returns:
        List of test cases in that category.
    """
    all_cases = load_test_cases()

    if category not in all_cases:
        available = ', '.join(all_cases.keys())
        raise ValueError(f"Category '{category}' not found. Available: {available}")

    return all_cases[category]


def get_test_case(category: str, name: str) -> Dict[str, Any]:
    """
    Get a specific test case by category and name.

    Args:
        category: Category name (e.g., 'overspeed_tests')
        name: Test case name (e.g., 'test_forward_overspeed_warning')

    Returns:
        Test case dictionary with 'name', 'description', 'params', and 'expected'.
    """
    cases = get_test_category(category)

    for case in cases:
        if case['name'] == name:
            return case

    available = ', '.join(c['name'] for c in cases)
    raise ValueError(f"Test case '{name}' not found in category '{category}'. "
                    f"Available: {available}")


def list_all_test_cases() -> None:
    """Print all available test cases."""
    all_cases = load_test_cases()

    print("Available test case categories:")
    print()

    for category, cases in all_cases.items():
        print(f"📁 {category}: {len(cases)} test(s)")
        for case in cases:
            print(f"   - {case['name']}: {case['description']}")
        print()


if __name__ == "__main__":
    # Demo usage
    list_all_test_cases()

    # Show example of loading a specific test
    print("\n" + "="*60)
    print("Example: Loading 'test_forward_overspeed_warning'")
    print("="*60)
    test = get_test_case('overspeed_tests', 'test_forward_overspeed_warning')
    print(f"\nName: {test['name']}")
    print(f"Description: {test['description']}")
    print(f"Parameters: {test['params']}")
    print(f"Expected: {test['expected']}")
    if 'notes' in test:
        print(f"Notes:\n{test['notes']}")
