"use client"

import { useState, useRef, useEffect } from "react"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Textarea } from "@/components/ui/textarea"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Checkbox } from "@/components/ui/checkbox"
import { useToast } from "@/hooks/use-toast"
import { Upload, X, Square, Trash2, Save, List } from "lucide-react"

interface BoundingBox {
  id: string
  x: number
  y: number
  width: number
  height: number
  label: string
  color: string
}

const OBJECT_TYPES = [
  "car",
  "motorcycle", 
  "pedestrian",
  "bicycle",
  "traffic_light",
  "traffic_sign",
  "truck",
  "bus"
]

const WEATHER_CONDITIONS = ["sunny", "rainy", "foggy", "night"]
const ROAD_TYPES = ["urban", "highway", "rural"]

const COLORS = [
  "#FF6B6B", "#4ECDC4", "#45B7D1", "#FFA07A", 
  "#98D8C8", "#F7DC6F", "#BB8FCE", "#85C1E2"
]

export default function DataCollectionPage() {
  const { toast } = useToast()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const imageRef = useRef<HTMLImageElement>(null)
  
  const [file, setFile] = useState<File | null>(null)
  const [preview, setPreview] = useState<string>("")
  const [selectedLabels, setSelectedLabels] = useState<string[]>([])
  const [weather, setWeather] = useState<string>("sunny")
  const [roadType, setRoadType] = useState<string>("urban")
  const [description, setDescription] = useState<string>("")
  const [boundingBoxes, setBoundingBoxes] = useState<BoundingBox[]>([])
  const [isDrawing, setIsDrawing] = useState(false)
  const [startPoint, setStartPoint] = useState<{ x: number; y: number } | null>(null)
  const [currentBox, setCurrentBox] = useState<BoundingBox | null>(null)
  const [selectedBoxLabel, setSelectedBoxLabel] = useState<string>("")
  const [isSubmitting, setIsSubmitting] = useState(false)
  const [datasetItems, setDatasetItems] = useState<any[]>([])
  const [showDatasetList, setShowDatasetList] = useState(false)

  useEffect(() => {
    fetchDatasetItems()
  }, [])

  useEffect(() => {
    if (preview && imageRef.current && canvasRef.current) {
      const img = imageRef.current
      const canvas = canvasRef.current
      const ctx = canvas.getContext("2d")
      
      if (ctx) {
        canvas.width = img.naturalWidth
        canvas.height = img.naturalHeight
        drawCanvas()
      }
    }
  }, [preview, boundingBoxes])

  const fetchDatasetItems = async () => {
    try {
      const response = await fetch("/api/dataset")
      if (response.ok) {
        const data = await response.json()
        setDatasetItems(data)
      }
    } catch (error) {
      console.error("Failed to fetch dataset:", error)
    }
  }

  const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const selectedFile = e.target.files?.[0]
    if (selectedFile) {
      if (selectedFile.size > 200 * 1024 * 1024) {
        toast({
          title: "File too large",
          description: "Maximum file size is 200MB",
          variant: "destructive"
        })
        return
      }
      
      setFile(selectedFile)
      const reader = new FileReader()
      reader.onloadend = () => {
        setPreview(reader.result as string)
        setBoundingBoxes([])
      }
      reader.readAsDataURL(selectedFile)
    }
  }

  const drawCanvas = () => {
    const canvas = canvasRef.current
    const ctx = canvas?.getContext("2d")
    const img = imageRef.current
    
    if (!ctx || !canvas || !img) return
    
    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    
    // Draw all bounding boxes
    boundingBoxes.forEach(box => {
      ctx.strokeStyle = box.color
      ctx.lineWidth = 3
      ctx.strokeRect(box.x, box.y, box.width, box.height)
      
      // Draw label
      ctx.fillStyle = box.color
      ctx.fillRect(box.x, box.y - 25, ctx.measureText(box.label).width + 10, 25)
      ctx.fillStyle = "white"
      ctx.font = "16px sans-serif"
      ctx.fillText(box.label, box.x + 5, box.y - 7)
    })
    
    // Draw current box while drawing
    if (currentBox) {
      ctx.strokeStyle = currentBox.color
      ctx.lineWidth = 3
      ctx.setLineDash([5, 5])
      ctx.strokeRect(currentBox.x, currentBox.y, currentBox.width, currentBox.height)
      ctx.setLineDash([])
    }
  }

  const handleCanvasMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!selectedBoxLabel) {
      toast({
        title: "Select object type",
        description: "Please select an object type before drawing",
        variant: "destructive"
      })
      return
    }
    
    const canvas = canvasRef.current
    if (!canvas) return
    
    const rect = canvas.getBoundingClientRect()
    const scaleX = canvas.width / rect.width
    const scaleY = canvas.height / rect.height
    
    const x = (e.clientX - rect.left) * scaleX
    const y = (e.clientY - rect.top) * scaleY
    
    setIsDrawing(true)
    setStartPoint({ x, y })
    setCurrentBox({
      id: Date.now().toString(),
      x,
      y,
      width: 0,
      height: 0,
      label: selectedBoxLabel,
      color: COLORS[boundingBoxes.length % COLORS.length]
    })
  }

  const handleCanvasMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!isDrawing || !startPoint || !currentBox) return
    
    const canvas = canvasRef.current
    if (!canvas) return
    
    const rect = canvas.getBoundingClientRect()
    const scaleX = canvas.width / rect.width
    const scaleY = canvas.height / rect.height
    
    const x = (e.clientX - rect.left) * scaleX
    const y = (e.clientY - rect.top) * scaleY
    
    const width = x - startPoint.x
    const height = y - startPoint.y
    
    setCurrentBox({
      ...currentBox,
      width,
      height
    })
    
    drawCanvas()
  }

  const handleCanvasMouseUp = () => {
    if (isDrawing && currentBox && Math.abs(currentBox.width) > 10 && Math.abs(currentBox.height) > 10) {
      // Normalize box (in case drawn right-to-left or bottom-to-top)
      const normalizedBox = {
        ...currentBox,
        x: currentBox.width < 0 ? currentBox.x + currentBox.width : currentBox.x,
        y: currentBox.height < 0 ? currentBox.y + currentBox.height : currentBox.y,
        width: Math.abs(currentBox.width),
        height: Math.abs(currentBox.height)
      }
      
      setBoundingBoxes([...boundingBoxes, normalizedBox])
      toast({
        title: "Bounding box added",
        description: `Added ${normalizedBox.label} detection`
      })
    }
    
    setIsDrawing(false)
    setStartPoint(null)
    setCurrentBox(null)
  }

  const handleDeleteBox = (id: string) => {
    setBoundingBoxes(boundingBoxes.filter(box => box.id !== id))
  }

  const handleSubmit = async () => {
    if (!file) {
      toast({
        title: "No file selected",
        description: "Please upload an image or video",
        variant: "destructive"
      })
      return
    }
    
    if (selectedLabels.length === 0) {
      toast({
        title: "No labels selected",
        description: "Please select at least one object type",
        variant: "destructive"
      })
      return
    }
    
    setIsSubmitting(true)
    
    try {
      const formData = new FormData()
      formData.append("file", file)
      
      const metadata = {
        labels: selectedLabels,
        boundingBoxes: boundingBoxes.map(box => ({
          x: box.x,
          y: box.y,
          width: box.width,
          height: box.height,
          label: box.label
        })),
        weather,
        roadType,
        description
      }
      
      formData.append("metadata", JSON.stringify(metadata))
      
      const response = await fetch("/api/dataset", {
        method: "POST",
        body: formData
      })
      
      if (!response.ok) {
        throw new Error("Failed to upload dataset")
      }
      
      const result = await response.json()
      
      toast({
        title: "Success!",
        description: `Dataset item created with ID: ${result.id}`
      })
      
      // Reset form
      setFile(null)
      setPreview("")
      setSelectedLabels([])
      setWeather("sunny")
      setRoadType("urban")
      setDescription("")
      setBoundingBoxes([])
      
      // Refresh dataset list
      fetchDatasetItems()
      
    } catch (error) {
      toast({
        title: "Upload failed",
        description: error instanceof Error ? error.message : "Unknown error",
        variant: "destructive"
      })
    } finally {
      setIsSubmitting(false)
    }
  }

  const handleDeleteDatasetItem = async (id: string) => {
    try {
      const response = await fetch(`/api/dataset/${id}`, {
        method: "DELETE"
      })
      
      if (response.ok) {
        toast({
          title: "Deleted",
          description: "Dataset item removed successfully"
        })
        fetchDatasetItems()
      }
    } catch (error) {
      toast({
        title: "Delete failed",
        description: "Could not delete dataset item",
        variant: "destructive"
      })
    }
  }

  return (
    <div className="container mx-auto p-6 max-w-7xl">
      <div className="flex justify-between items-center mb-6">
        <div>
          <h1 className="text-3xl font-bold">Data Collection</h1>
          <p className="text-muted-foreground">Collect and annotate training data for ADAS models</p>
        </div>
        <Button 
          variant="outline"
          onClick={() => setShowDatasetList(!showDatasetList)}
        >
          <List className="h-4 w-4 mr-2" />
          {showDatasetList ? "Hide" : "Show"} Dataset ({datasetItems.length})
        </Button>
      </div>

      {showDatasetList && (
        <Card className="mb-6">
          <CardHeader>
            <CardTitle>Collected Dataset</CardTitle>
            <CardDescription>All annotated images and videos</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-2">
              {datasetItems.map((item) => (
                <div key={item.id} className="flex justify-between items-center p-3 border rounded-lg">
                  <div>
                    <p className="font-medium">{item.filePath}</p>
                    <p className="text-sm text-muted-foreground">
                      {item.labels?.join(", ")} • {item.boundingBoxes?.length || 0} boxes • {item.weather} • {item.roadType}
                    </p>
                  </div>
                  <Button
                    variant="destructive"
                    size="sm"
                    onClick={() => handleDeleteDatasetItem(item.id)}
                  >
                    <Trash2 className="h-4 w-4" />
                  </Button>
                </div>
              ))}
              {datasetItems.length === 0 && (
                <p className="text-center text-muted-foreground py-4">No dataset items yet</p>
              )}
            </div>
          </CardContent>
        </Card>
      )}

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Left Column - Upload & Annotation */}
        <Card>
          <CardHeader>
            <CardTitle>Upload & Annotate</CardTitle>
            <CardDescription>Upload image/video and draw bounding boxes</CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            {/* File Upload */}
            <div>
              <Label htmlFor="file">Image or Video</Label>
              <Input
                id="file"
                type="file"
                accept="image/*,video/*"
                onChange={handleFileChange}
                className="cursor-pointer"
              />
            </div>

            {/* Preview & Canvas */}
            {preview && (
              <div className="space-y-2">
                <Label>Draw Bounding Boxes</Label>
                <div className="relative border rounded-lg overflow-hidden bg-black">
                  <img
                    ref={imageRef}
                    src={preview}
                    alt="Preview"
                    className="w-full h-auto"
                    style={{ display: "block" }}
                  />
                  <canvas
                    ref={canvasRef}
                    className="absolute top-0 left-0 w-full h-full cursor-crosshair"
                    onMouseDown={handleCanvasMouseDown}
                    onMouseMove={handleCanvasMouseMove}
                    onMouseUp={handleCanvasMouseUp}
                    onMouseLeave={handleCanvasMouseUp}
                  />
                </div>
                
                {/* Drawing Instructions */}
                <div className="flex items-center gap-2 text-sm text-muted-foreground">
                  <Square className="h-4 w-4" />
                  <span>Select object type below, then click and drag to draw bounding box</span>
                </div>

                {/* Current Box Label Selector */}
                <div>
                  <Label>Drawing Label</Label>
                  <Select value={selectedBoxLabel} onValueChange={setSelectedBoxLabel}>
                    <SelectTrigger>
                      <SelectValue placeholder="Select object type to draw" />
                    </SelectTrigger>
                    <SelectContent>
                      {OBJECT_TYPES.map((type) => (
                        <SelectItem key={type} value={type}>
                          {type}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                </div>

                {/* Bounding Boxes List */}
                {boundingBoxes.length > 0 && (
                  <div>
                    <Label>Bounding Boxes ({boundingBoxes.length})</Label>
                    <div className="space-y-1 mt-2 max-h-40 overflow-y-auto">
                      {boundingBoxes.map((box) => (
                        <div
                          key={box.id}
                          className="flex justify-between items-center p-2 border rounded"
                          style={{ borderColor: box.color }}
                        >
                          <span className="text-sm">
                            <span style={{ color: box.color }} className="font-bold">■</span> {box.label}
                          </span>
                          <Button
                            variant="ghost"
                            size="sm"
                            onClick={() => handleDeleteBox(box.id)}
                          >
                            <X className="h-4 w-4" />
                          </Button>
                        </div>
                      ))}
                    </div>
                  </div>
                )}
              </div>
            )}
          </CardContent>
        </Card>

        {/* Right Column - Metadata */}
        <Card>
          <CardHeader>
            <CardTitle>Metadata</CardTitle>
            <CardDescription>Provide context about the scene</CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            {/* Object Types */}
            <div>
              <Label>Object Types (Multi-select)</Label>
              <div className="grid grid-cols-2 gap-2 mt-2">
                {OBJECT_TYPES.map((type) => (
                  <div key={type} className="flex items-center space-x-2">
                    <Checkbox
                      id={type}
                      checked={selectedLabels.includes(type)}
                      onCheckedChange={(checked) => {
                        if (checked) {
                          setSelectedLabels([...selectedLabels, type])
                        } else {
                          setSelectedLabels(selectedLabels.filter(l => l !== type))
                        }
                      }}
                    />
                    <label
                      htmlFor={type}
                      className="text-sm font-medium leading-none peer-disabled:cursor-not-allowed peer-disabled:opacity-70 cursor-pointer"
                    >
                      {type}
                    </label>
                  </div>
                ))}
              </div>
            </div>

            {/* Weather Condition */}
            <div>
              <Label htmlFor="weather">Weather Condition</Label>
              <Select value={weather} onValueChange={setWeather}>
                <SelectTrigger id="weather">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  {WEATHER_CONDITIONS.map((condition) => (
                    <SelectItem key={condition} value={condition}>
                      {condition}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </div>

            {/* Road Type */}
            <div>
              <Label htmlFor="roadType">Road Type</Label>
              <Select value={roadType} onValueChange={setRoadType}>
                <SelectTrigger id="roadType">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  {ROAD_TYPES.map((type) => (
                    <SelectItem key={type} value={type}>
                      {type}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </div>

            {/* Description */}
            <div>
              <Label htmlFor="description">Description (Optional)</Label>
              <Textarea
                id="description"
                placeholder="Additional notes about this scene..."
                value={description}
                onChange={(e) => setDescription(e.target.value)}
                rows={4}
              />
            </div>

            {/* Submit Button */}
            <Button
              className="w-full"
              onClick={handleSubmit}
              disabled={!file || selectedLabels.length === 0 || isSubmitting}
            >
              {isSubmitting ? (
                <>
                  <Upload className="h-4 w-4 mr-2 animate-spin" />
                  Uploading...
                </>
              ) : (
                <>
                  <Save className="h-4 w-4 mr-2" />
                  Submit to Dataset
                </>
              )}
            </Button>

            {/* Summary */}
            <div className="p-4 bg-muted rounded-lg space-y-1 text-sm">
              <p><strong>Summary:</strong></p>
              <p>File: {file ? file.name : "Not selected"}</p>
              <p>Labels: {selectedLabels.length || 0}</p>
              <p>Bounding Boxes: {boundingBoxes.length || 0}</p>
              <p>Weather: {weather}</p>
              <p>Road: {roadType}</p>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  )
}
