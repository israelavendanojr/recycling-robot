import { Component, ReactNode } from 'react'

type Props = { children: ReactNode }
type State = { hasError: boolean; error?: any }

export class ErrorBoundary extends Component<Props, State> {
  state: State = { hasError: false }
  static getDerivedStateFromError(error: any) { return { hasError: true, error } }
  componentDidCatch(error: any, info: any) { console.error('App crashed:', error, info) }
  render() {
    if (this.state.hasError) {
      return (
        <div style={{ padding: 16 }}>
          <h2>Something went wrong.</h2>
          <p>Check console for details.</p>
        </div>
      )
    }
    return this.props.children
  }
}

export default ErrorBoundary


