const js = require('@eslint/js')
const globals = require('globals')
const reactHooks = require('eslint-plugin-react-hooks')
const reactRefresh = require('eslint-plugin-react-refresh')
const tseslint = require('@typescript-eslint/eslint-plugin')
const tsparser = require('@typescript-eslint/parser')

module.exports = [
  { ignores: ['dist'] },
  {
    files: ['**/*.{ts,tsx}'],
    languageOptions: {
      ecmaVersion: 2020,
      globals: globals.browser,
      parser: tsparser,
    },
    plugins: {
      '@typescript-eslint': tseslint,
      'react-hooks': reactHooks,
      'react-refresh': reactRefresh,
    },
    rules: {
      ...js.configs.recommended.rules,
      ...tseslint.configs.recommended.rules,
      ...reactHooks.configs.recommended.rules,
      'react-refresh/only-export-components': [
        'warn',
        { allowConstantExport: true },
      ],
      // Enforce use of theme tokens instead of arbitrary hex colors
      'no-restricted-syntax': [
        'error',
        {
          selector: 'Literal[value=/^#(?!f8fafc|f1f5f9|ffffff|e5e7eb|1f2937|6b7280|374151|0e7490|0891b2|06b6d4|67e8f9|a5f3fc)/i]',
          message: 'Use theme tokens from tokens.ts instead of arbitrary hex colors',
        },
      ],
    },
  },
]
