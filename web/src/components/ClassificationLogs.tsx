import React, { useState, useMemo } from 'react'
import { format } from 'date-fns'
import { useClassifications } from '../hooks/useClassifications'
import type { Classification } from '../types'

interface PaginationProps {
  currentPage: number
  totalPages: number
  pageSize: number
  totalItems: number
  onPageChange: (page: number) => void
  onPageSizeChange: (size: number) => void
}

const Pagination: React.FC<PaginationProps> = ({
  currentPage,
  totalPages,
  pageSize,
  totalItems,
  onPageChange,
  onPageSizeChange,
}) => (
  <div className="flex items-center justify-between mt-4 text-sm">
    <div className="flex items-center space-x-2">
      <span className="text-brand-text-label">Items per page:</span>
      <select
        value={pageSize}
        onChange={(e) => onPageSizeChange(Number(e.target.value))}
        className="border border-brand-border rounded px-2 py-1 text-brand-text-primary focus:outline-none focus:ring-2 focus:ring-brand-teal-medium"
      >
        <option value={10}>10</option>
        <option value={25}>25</option>
        <option value={50}>50</option>
      </select>
    </div>

    <div className="text-brand-text-secondary">
      Showing {Math.min((currentPage - 1) * pageSize + 1, totalItems)} to{' '}
      {Math.min(currentPage * pageSize, totalItems)} of {totalItems} entries
    </div>

    <div className="flex items-center space-x-1">
      <button
        onClick={() => onPageChange(currentPage - 1)}
        disabled={currentPage === 1}
        className="btn btn-secondary disabled:opacity-50 disabled:cursor-not-allowed"
      >
        Previous
      </button>
      
      <div className="flex items-center space-x-1 mx-2">
        {Array.from({ length: Math.min(5, totalPages) }, (_, i) => {
          const page = i + 1
          return (
            <button
              key={page}
              onClick={() => onPageChange(page)}
              className={`px-2 py-1 rounded text-sm ${
                currentPage === page
                  ? 'bg-brand-teal-medium text-white'
                  : 'text-brand-text-primary hover:bg-gray-100'
              }`}
            >
              {page}
            </button>
          )
        })}
      </div>

      <button
        onClick={() => onPageChange(currentPage + 1)}
        disabled={currentPage === totalPages}
        className="btn btn-secondary disabled:opacity-50 disabled:cursor-not-allowed"
      >
        Next
      </button>
    </div>
  </div>
)

const exportToCSV = (data: Classification[], filename: string = 'classifications.csv') => {
  const headers = ['ID', 'Timestamp', 'Local Time', 'Label', 'Confidence', 'Image Source']
  const rows = data.map(item => [
    item.id,
    item.timestamp,
    format(new Date(item.timestamp * 1000), 'yyyy-MM-dd HH:mm:ss'),
    item.label,
    (item.confidence * 100).toFixed(2) + '%',
    item.image_source,
  ])

  const csvContent = [headers, ...rows]
    .map(row => row.map(field => `"${field}"`).join(','))
    .join('\n')

  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' })
  const link = document.createElement('a')
  const url = URL.createObjectURL(blob)
  link.setAttribute('href', url)
  link.setAttribute('download', filename)
  link.style.visibility = 'hidden'
  document.body.appendChild(link)
  link.click()
  document.body.removeChild(link)
}

export const ClassificationLogs: React.FC = () => {
  const { classifications, loading, error } = useClassifications()
  const [searchTerm, setSearchTerm] = useState('')
  const [currentPage, setCurrentPage] = useState(1)
  const [pageSize, setPageSize] = useState(25)

  // Filter and sort classifications
  const filteredClassifications = useMemo(() => {
    return classifications
      .filter(item => 
        item.label.toLowerCase().includes(searchTerm.toLowerCase())
      )
      .sort((a, b) => b.timestamp - a.timestamp)
  }, [classifications, searchTerm])

  // Pagination
  const totalPages = Math.ceil(filteredClassifications.length / pageSize)
  const paginatedData = useMemo(() => {
    const start = (currentPage - 1) * pageSize
    return filteredClassifications.slice(start, start + pageSize)
  }, [filteredClassifications, currentPage, pageSize])

  // Reset page when search changes
  React.useEffect(() => {
    setCurrentPage(1)
  }, [searchTerm])

  const handleExportFiltered = () => {
    exportToCSV(filteredClassifications, `classifications_filtered_${Date.now()}.csv`)
  }

  const handleExportAll = () => {
    exportToCSV(classifications, `classifications_all_${Date.now()}.csv`)
  }

  if (loading) {
    return (
      <div className="card">
        <div className="text-lg font-semibold text-brand-text-primary mb-6">
          Classification Logs
        </div>
        <div className="space-y-3">
          {[...Array(10)].map((_, i) => (
            <div key={i} className="h-8 bg-gray-200 rounded animate-pulse"></div>
          ))}
        </div>
      </div>
    )
  }

  if (error) {
    return (
      <div className="card bg-red-50 border-red-200">
        <div className="text-red-600 font-medium">Classification Logs Error</div>
        <div className="text-red-500 text-sm mt-1">{error}</div>
      </div>
    )
  }

  return (
    <div className="card">
      <div className="flex justify-between items-center mb-6">
        <div className="text-lg font-semibold text-brand-text-primary">
          Classification Logs
        </div>
        <div className="flex items-center space-x-2">
          <button
            onClick={handleExportFiltered}
            className="btn btn-secondary text-xs"
            disabled={filteredClassifications.length === 0}
          >
            Export Filtered ({filteredClassifications.length})
          </button>
          <button
            onClick={handleExportAll}
            className="btn btn-primary text-xs"
            disabled={classifications.length === 0}
          >
            Export All ({classifications.length})
          </button>
        </div>
      </div>

      {/* Search */}
      <div className="mb-4">
        <input
          type="text"
          placeholder="Search by label..."
          value={searchTerm}
          onChange={(e) => setSearchTerm(e.target.value)}
          className="w-full max-w-md px-3 py-2 border border-brand-border rounded-md focus:outline-none focus:ring-2 focus:ring-brand-teal-medium"
        />
      </div>

      {/* Table */}
      <div className="overflow-x-auto">
        <table 
          className="w-full border-collapse"
          role="table"
          aria-label="Classification logs"
        >
          <thead className="sticky top-0 bg-gray-50 border-b border-brand-border">
            <tr>
              <th className="text-left py-3 px-4 font-medium text-brand-text-label">ID</th>
              <th className="text-left py-3 px-4 font-medium text-brand-text-label">Local Time</th>
              <th className="text-left py-3 px-4 font-medium text-brand-text-label">Label</th>
              <th className="text-left py-3 px-4 font-medium text-brand-text-label">Confidence</th>
              <th className="text-left py-3 px-4 font-medium text-brand-text-label">Image Source</th>
            </tr>
          </thead>
          <tbody>
            {paginatedData.map((item, index) => (
              <tr
                key={item.id}
                className={`border-b border-brand-border ${
                  index % 2 === 0 ? 'bg-white' : 'bg-gray-50'
                } hover:bg-gray-100 focus-within:bg-gray-100`}
                tabIndex={0}
                role="row"
              >
                <td className="py-3 px-4 text-brand-text-primary font-mono text-sm">
                  {item.id}
                </td>
                <td className="py-3 px-4 text-brand-text-primary">
                  {format(new Date(item.timestamp * 1000), 'MM/dd HH:mm:ss')}
                </td>
                <td className="py-3 px-4 text-brand-text-primary font-medium capitalize">
                  {item.label}
                </td>
                <td className="py-3 px-4 text-brand-teal-medium font-semibold">
                  {(item.confidence * 100).toFixed(1)}%
                </td>
                <td className="py-3 px-4 text-brand-text-secondary text-sm">
                  {item.image_source}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      {filteredClassifications.length === 0 && (
        <div className="text-center py-8 text-brand-text-secondary">
          {searchTerm ? 'No classifications match your search.' : 'No classifications available.'}
        </div>
      )}

      {filteredClassifications.length > 0 && (
        <Pagination
          currentPage={currentPage}
          totalPages={totalPages}
          pageSize={pageSize}
          totalItems={filteredClassifications.length}
          onPageChange={setCurrentPage}
          onPageSizeChange={(size) => {
            setPageSize(size)
            setCurrentPage(1)
          }}
        />
      )}
    </div>
  )
}
