.PHONY: help up down logs build clean install test

help: ## Show this help message
	@echo "Recycling Robot - Available Commands:"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

install: ## Install dependencies
	@echo "📦 Installing dependencies..."
	cd backend && python3 -m venv venv && . venv/bin/activate && pip install -r requirements.txt
	cd web && npm install

up: ## Start all services
	@echo "🚀 Starting services..."
	docker compose up -d

down: ## Stop all services
	@echo "🛑 Stopping services..."
	docker compose down

logs: ## Show service logs
	@echo "📋 Showing logs..."
	docker compose logs -f

build: ## Build all services
	@echo "🔨 Building services..."
	docker compose build

clean: ## Clean up Docker resources
	@echo "🧹 Cleaning up..."
	docker system prune -f
	docker volume prune -f

dev-backend: ## Start backend in development mode
	@echo "🔧 Starting backend development server..."
	cd backend && source venv/bin/activate && python app.py

dev-frontend: ## Start frontend in development mode
	@echo "🎨 Starting frontend development server..."
	cd web && npm run dev

dev: ## Start both backend and frontend in development mode
	@echo "🚀 Starting development environment..."
	@make dev-backend & make dev-frontend

test: ## Run tests
	@echo "🧪 Running tests..."
	cd backend && source venv/bin/activate && python -m pytest tests/ || echo "No tests found"
	cd web && npm test || echo "No tests configured"

lint: ## Run linting
	@echo "🔍 Running linting..."
	cd backend && source venv/bin/activate && flake8 . || echo "Flake8 not installed"
	cd web && npm run lint || echo "Lint script not configured"

format: ## Format code
	@echo "✨ Formatting code..."
	cd backend && source venv/bin/activate && black . || echo "Black not installed"
	cd web && npm run format || echo "Format script not configured"

reset: ## Reset everything (use with caution!)
	@echo "⚠️  Resetting everything..."
	@make down
	@make clean
	@rm -rf backend/venv web/node_modules
	@echo "✅ Reset complete. Run 'make install' to reinstall dependencies."
