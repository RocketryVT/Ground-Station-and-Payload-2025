# Tauri + SvelteKit + TypeScript

This template should help get you started developing with Tauri, SvelteKit and TypeScript in Vite.

Tauri is an alternative to Electron for building desktop applications with web technologies. It's a smaller binary and has a smaller memory footprint than Electron. It bundles a webview with a Rust application that can be used to interact with the host system. Meaning that it will have a consistent look and feel across all platforms.

SvelteKit is a framework for building web applications with Svelte, a UI framework, and has a file-based routing system.

TypeScript is a superset of JavaScript that adds static types to the language. It helps catch errors before they happen and makes the code easier to read and understand.

Vite is a build tool that focuses on speed and simplicity. It compiles and minimizes TypeScript, JavaScript, HTML, and CSS files.

## Recommended IDE Setup

[VS Code](https://code.visualstudio.com/) + [Svelte](https://marketplace.visualstudio.com/items?itemName=svelte.svelte-vscode) + [Tauri](https://marketplace.visualstudio.com/items?itemName=tauri-apps.tauri-vscode) + [rust-analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer).

## Get started

You need several packages to be installed on your system to be able to build the project:

- rust
- deno

## Running the project

```bash
deno task tauri dev
```

## Building the project

```bash
deno task tauri build
```

Building the project will take a while, but once it's done, you can find the executable in the `src-tauri/target/release/bundle` directory.
