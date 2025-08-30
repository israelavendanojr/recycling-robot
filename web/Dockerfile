FROM node:20

WORKDIR /app

# Copy package files
COPY package.json package-lock.json* ./

# Install dependencies
RUN npm install

# Copy source code
COPY . .

# Expose port 5173
EXPOSE 5173

# Start development server
CMD ["npm", "run", "dev"]
