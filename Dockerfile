# ── Build stage ───────────────────────────────────────────────────────────────
FROM rust:1.85-slim AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
    && rm -rf /var/lib/apt/lists/*

RUN rustup target add wasm32-unknown-unknown \
    && cargo install trunk --locked

WORKDIR /app

# Cache dependencies before copying source
COPY Cargo.toml Cargo.lock ./
RUN mkdir -p src && echo "fn main(){}" > src/main.rs \
    && cargo fetch

COPY . .
RUN trunk build --release

# ── Serve stage ───────────────────────────────────────────────────────────────
FROM nginx:alpine

COPY --from=builder /app/dist /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf

EXPOSE 80
