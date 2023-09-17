# Targets without a command associated are just considered to be a dependency for other targets
.PHONY: all clean run scale

# Default target
all: clean run

# A clean target to remove generated files and Docker volumes
clean:
	@echo "Cleaning environment..."
	docker-compose down -v
	docker-compose rm -f

# A target to bring up Docker containers
run:
	@echo "Launching docker containers..."
	docker-compose up

# Scale our sensor_mocker
scale:
	@echo "Running additional sensors..."
	docker-compose up --scale sensor_mocker=3 -d