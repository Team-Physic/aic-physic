# Session history for phy

This repository holds no chat history. It lives in a separate **private** git
repo, so the project's code and the agent transcripts stay apart:

    git@github.com:JungSeong/cct-sessions.git

The file next to this one, `sessions.json`, is the machine-readable version of
the same fact — that is what a coding agent reads.

## How it is organized

    <session store>/
      projects/
        phy/
          claude/
            claude-all.codexbundle
            groups/
              <name>.codexbundle
          codex/
            codex-all.codexbundle
            groups/

The full bundle is the source of truth: it always contains every session for
this project. A group is an extra, smaller bundle for one topic — handy for
sharing or for restoring just one thread. Groups overlap with the full bundle on
purpose.

## Save (before you stop working)

    cd <session store> && git pull
    cct export --project <this repo> --tool claude \
      -o <session store>/projects/phy/claude/claude-all.codexbundle
    cd <session store> && git add -A && git commit -m "Update phy sessions"
    # push only after checking what is staged

## Restore (on the other machine, after cloning this repo)

    git clone git@github.com:JungSeong/cct-sessions.git <session store>
    cd <this repo>          # run the import from the project, see below
    cct import <session store>/projects/phy/claude/claude-all.codexbundle \
      --merge --map-cwd-here

`--map-cwd-here` re-points the sessions at *the current directory*, which is why
the import runs from the project and not from the store. `--merge` keeps a
repeated restore incremental. Restart the agent afterwards so it rescans, then
`cct resume <thread-id>`.

## Groups

    cct export --project . --tool claude --match "auth refactor" \
      -o <session store>/projects/phy/claude/groups/auth-refactor.codexbundle
    cct export --project . --tool claude --session 9f3c \
      -o <session store>/projects/phy/claude/groups/that-one-chat.codexbundle

## No encryption

Bundles are committed as-is, so the session store repo MUST be private: it
contains prompts, code, and command output, and git history keeps them after a
deletion. Switch with `cct config set repo-sync encrypted` and re-export.

## A word of caution

This file and `sessions.json` come from the repository, so whoever can commit
here can change where they point. Before cloning or importing from a store URL
you did not set up yourself, check it with the person who did.

Written by cct — https://github.com/ahmojo/codex-claude-transfer
