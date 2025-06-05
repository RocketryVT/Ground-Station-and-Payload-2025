// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::thread;
use std::process::Command;
use crossterm::{
    execute,
    terminal::{enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{backend::CrosstermBackend, Terminal};

fn main() {
    // Spawn a new terminal window
    if cfg!(target_os = "macos") {
        Command::new("open")
            .arg("-a")
            .arg("Terminal")
            .spawn()
            .expect("Failed to open a new terminal window");
    }

    // Spawn a thread for the TUI
    std::thread::spawn(|| {
        // Initialize the terminal
        crossterm::terminal::enable_raw_mode().expect("Failed to enable raw mode");
        let mut stdout = std::io::stdout();
        crossterm::execute!(stdout, crossterm::terminal::EnterAlternateScreen)
            .expect("Failed to enter alternate screen");
        let backend = ratatui::backend::CrosstermBackend::new(stdout);
        let mut terminal = ratatui::Terminal::new(backend).expect("Failed to create terminal");

        // Run the TUI loop
        if let Err(err) = run_tui(&mut terminal) {
            eprintln!("Error running TUI: {:?}", err);
        }

        // Restore the terminal
        crossterm::execute!(terminal.backend_mut(), crossterm::terminal::LeaveAlternateScreen)
            .expect("Failed to leave alternate screen");
    });

    // Run the Tauri application
    rocketry_tauri_lib::run();
}

fn run_tui<B: ratatui::backend::Backend>(terminal: &mut Terminal<B>) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        terminal.draw(|f| {
            let size = f.size();
            let block = ratatui::widgets::Block::default().title("TUI").borders(ratatui::widgets::Borders::ALL);
            f.render_widget(block, size);
        })?;
        // Add logic to break the loop or handle input if needed
    }
}
