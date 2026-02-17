.PHONY: up down logs share test build clean

up:
	docker compose up -d

down:
	docker compose down

build:
	docker compose build

logs:
	docker compose logs -f

logs-backend:
	docker compose logs -f backend worker

share:
	docker compose --profile share up -d

test:
	docker compose exec backend pytest -v

clean:
	docker compose down -v --remove-orphans

migrate:
	docker compose exec backend alembic upgrade head

shell:
	docker compose exec backend python -c "import IPython; IPython.start_ipython()" 2>/dev/null || docker compose exec backend python

restart:
	docker compose restart $(service)
