# *** BUILDER
# The builder image, used to build the virtual environment
FROM python:3.10-slim as builder

# Set the working directory in the container
WORKDIR /app
ENV PYTHONPATH=${PYTHONPATH}:${PWD}

# Copy the requirements
COPY pyproject.toml ./

# Install any needed packages specified in requirements.txt
RUN pip install --no-cache-dir --upgrade pip poetry setuptools

# Install project dependencies using poetry
ENV POETRY_NO_INTERACTION=1 \
    POETRY_VIRTUALENVS_IN_PROJECT=1 \
    POETRY_VIRTUALENVS_CREATE=1 \
    POETRY_CACHE_DIR=/tmp/poetry_cache
RUN poetry config virtualenvs.create false
RUN poetry install --only main --no-root  && rm -rf $POETRY_CACHE_DIR

# *** RUNTIME
# A basic docker container for mocking sensor data
FROM python:3.10-slim as runtime

# Define environment variable (optional)
ENV MQTT_HOST "mosquitto"
ENV MQTT_PORT 1883
ENV MQTT_TOPICS "sensors/bedroom/temperature, sensors/kitchen/temperature, sensors/mancave/temperature"

# Set the working directory in the container
WORKDIR /app
ENV PYTHONPATH=${PYTHONPATH}:${PWD}
ENV VIRTUAL_ENV=/app/.venv \
    PATH="/app/.venv/bin:$PATH"

COPY --from=builder ${VIRTUAL_ENV} ${VIRTUAL_ENV}

# COPY script
COPY mocker.py ./

# Run your script when the container launches
CMD ["python", "mocker.py"]