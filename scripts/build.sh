# 1) Build the React app (already done above, but repeating for clarity)
cd frontend
npm run build

# 2) Back to repo root
cd ..

# 3) Bring up containers (from repo root!)
docker compose up --build -d

# 4) Sanity check
docker compose ps
