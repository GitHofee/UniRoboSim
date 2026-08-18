PYTHON ?= python

.PHONY: build format lint test type verify

format:
	$(PYTHON) -m ruff format src tests
	$(PYTHON) -m ruff check --fix src tests

lint:
	$(PYTHON) -m ruff format --check src tests
	$(PYTHON) -m ruff check src tests

type:
	$(PYTHON) -m mypy src

test:
	$(PYTHON) -m pytest --cov=unirobosim --cov-branch --cov-report=term-missing

build:
	$(PYTHON) -m build --no-isolation

verify: lint type test
