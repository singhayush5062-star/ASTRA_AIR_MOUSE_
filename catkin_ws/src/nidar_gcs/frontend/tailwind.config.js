/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,ts,jsx,tsx}'],
  theme: {
    extend: {
      colors: {
        // ── Base backgrounds ──
        bg:        '#09090B',
        surface:   '#111113',
        surface2:  '#18181B',
        border:    '#27272A',

        // ── Text ──
        text:      '#E4E4E7',
        muted:     '#A1A1AA',
        disabled:  '#52525B',

        // ── Semantic accent ──
        cyan:   '#00F0FF',
        amber:  '#FFB000',
        green:  '#00FF41',
        red:    '#FF003C',
        orange: '#FF5500',
      },
      fontFamily: {
        ui:   ['IBM Plex Sans', 'system-ui', 'sans-serif'],
        mono: ['JetBrains Mono', 'Fira Code', 'monospace'],
      },
      fontSize: {
        '2xs': ['0.625rem', { lineHeight: '0.875rem' }],
      },
      borderColor: {
        DEFAULT: '#27272A',
      },
      keyframes: {
        pulse_slow: {
          '0%, 100%': { opacity: '1' },
          '50%':       { opacity: '0.4' },
        },
        scan: {
          '0%':   { transform: 'translateY(-100%)' },
          '100%': { transform: 'translateY(100%)' },
        },
        blink: {
          '0%, 100%': { opacity: '1' },
          '50%':       { opacity: '0' },
        },
      },
      animation: {
        pulse_slow: 'pulse_slow 2s ease-in-out infinite',
        scan:       'scan 3s linear infinite',
        blink:      'blink 1s step-end infinite',
      },
    },
  },
  plugins: [],
};
