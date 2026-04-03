# tobas_wiki_ja

## Setup (Ubuntu 24.04 LTS)

```bash
$ python -m venv .venv
$ source .venv/bin/activate
$ pip install --upgrade pip
$ pip install mkdocs mkdocs-material mike
```

## Test locally

1. Start the MkDocs server.

```bash
$ mkdocs serve
```

2. Then open http://127.0.0.1:8000/ in your browser.

## Deploy

TODO

```bash
$ mike deploy --push --update-aliases x.x latest
$ mike set-default --push latest
$ mike serve
```
