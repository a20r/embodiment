PY := python3

.PHONY: build smoke run dashboard report clean

build:
	docker build -t mazebot-bot - < Dockerfile.bot

smoke: build
	$(PY) -m scripts.smoke

run:
	./botctl run --config config.yaml

dashboard:
	./botctl dashboard --config config.yaml

report:
	./botctl report --config config.yaml

clean:
	rm -rf runs/smoke runs/hosttest runs/followdbg
