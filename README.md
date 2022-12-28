# My Lulu Keyboard Firmware

My firmware for the [Lulu Keyboard](https://boardsource.xyz/store/61d0b772319a1f3cc53ba2fb) written in Rust

It is generally structured so customization takes place within `layout.rs` for keyboard functionality and `config.rs` for constants.

## Flash Steps

1. `cargo install elf2uf2-rs` (first time)
2. Put the board into UF2 flashing mode
3. `cargo run --release`
