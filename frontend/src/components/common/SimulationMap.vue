<template>
  <div class="map-conatiner" :style="{ width: size + 'px', height: size + 'px'}">
    <canvas class="map-canvas-path" id="map-path" @click="clickMap" width="350" height="350"></canvas>
    <canvas class="map-canvas-pos" id="map-pos" width="350" height="350"></canvas>
    <canvas class="map-canvas-grid" id="map-grid" width="350" height="350"></canvas>
    <canvas class="map-canvas-application" width="350" height="350"></canvas>
  </div>
</template>
<style lang="scss" scoped>
  @import './SimulationMap.scss';
</style>
<script>
import axios from 'axios'
export default {
  name: 'SimulationMap',
  props: {
    clickEnable: {
      type: Boolean,
      default: true,
    },
    size: {
      type: String,
      default: '150',
    }
  },
  data() {
    return {
      isClicked: false,
      isProcessing: false,
      isTaken: false,
      isMapUpdated: false,
      pastX: null,
      pastY: null,
      mapData: null,
      appPos: {
        tv: [155, 129],
        airCon: [232, 97] 
      }
    }
  },
  mounted() {
    const canvasTag = document.querySelector('#map-grid')
    const posCanvasTag = document.querySelector('#map-pos')
    const appCanvasTag = document.querySelector('.map-canvas-application')
    const context = canvasTag.getContext('2d')
    const posContext = posCanvasTag.getContext('2d')
    const appContext = appCanvasTag.getContext('2d')
    const imageData = context.createImageData(350, 350)
    const posImageData = posContext.createImageData(350, 350)
    const w = 350
    const h = 350
    
    for (let app in this.appPos) {
      const [x, y] = this.appPos[app]
      appContext.font = '40px serif bold'
      appContext.fillStyle = 'black'
      appContext.fillText(app, x, y)
    }

    this.$socket.on('turtleBotPos', message => {
      if (!this.isMapUpdated) return
      const data = JSON.parse(message)
      const x = data.x
      const y = data.y
      
      for (let dx = 0; dx < 10; dx++) {
        for (let dy = 0; dy < 10; dy++) {
          const curIdx = ((y + dy) * h + (x + dx)) * 4
          const pastIdx = ((this.pastY + dy) * h + (this.pastX + dx)) * 4
          if (0 <= pastIdx < w * h * 4 && pastIdx !== curIdx) {
            posImageData.data[pastIdx + 0] = 0
            posImageData.data[pastIdx + 1] = 0
            posImageData.data[pastIdx + 2] = 0
            posImageData.data[pastIdx + 3] = 0
          }

          if (0 <= curIdx < w * h * 4) {
            posImageData.data[curIdx + 0] = 255
            posImageData.data[curIdx + 1] = 0
            posImageData.data[curIdx + 2] = 0
            posImageData.data[curIdx + 3] = 255
          }
        }
      }
      this.pastX = x
      this.pastY = y
      posContext.putImageData(posImageData, 0, 0)
    })

    this.$socket.on('responsePathFromRos', message => {
      const pathContext = document.querySelector('#map-path').getContext('2d')
      this.isClicked = false
      pathContext.clearRect(0, 0, w, h)
      pathContext.fillStyle = 'green'
      const path = JSON.parse(message)
      path.forEach(node => {
        const x = node[0]
        const y = node[1]
        pathContext.fillRect(x, y, 2, 2)
      })
      this.isProcessing = false
    })

    axios('http://localhost:3000/map1', { method: 'GET'})
      .then(res => {
        const grid = res.data.split(' ')
        for (let i = 0; i < w * h; i++) {
            const idx = i * 4
            if (parseInt(grid[i]) === 50) {
              imageData.data[idx + 0] = 128 
              imageData.data[idx + 1] = 128 
              imageData.data[idx + 2] = 128 
              imageData.data[idx + 3] = 127 
            } else if (parseInt(grid[i]) > 100) {
              imageData.data[idx + 0] = 0 
              imageData.data[idx + 1] = 128 
              imageData.data[idx + 2] = 0 
              imageData.data[idx + 3] = 127 
            } else if (parseInt(grid[i]) >= 30) {
              imageData.data[idx + 0] = 0 
              imageData.data[idx + 1] = 0
              imageData.data[idx + 2] = 0 
              imageData.data[idx + 3] = 255 
            } else {
              imageData.data[idx + 0] = 240 
              imageData.data[idx + 1] = 240
              imageData.data[idx + 2] = 240
              imageData.data[idx + 3] = 128 
            }
        }
        context.putImageData(imageData, 0, 0)
        this.isMapUpdated = true
      })
  },
  methods: {
    async clickMap (event) {
      if (this.isProcessing || !this.clickEnable) return
      const posContext = document.querySelector('#map-path').getContext('2d')
      const widthRatio = 350 / event.target.scrollWidth
      const heightRatio = 350 / event.target.scrollHeight
      const x = Math.round(event.layerX * widthRatio)
      const y = Math.round(event.layerY * heightRatio)
      
      if (!this.isClicked) {
        this.start = [x, y]
        this.isClicked = true
        posContext.font = '20px serif bold'
        posContext.fillStyle = 'green'
        posContext.fillText('START', x - 5, y - 5)
        posContext.fillRect(x, y, 5, 5)
      } else if (this.isClicked) {
        this.isProcessing = true
        this.end = [x, y]
        posContext.font = '20px serif bold'
        posContext.fillStyle = 'black'
        posContext.fillText('END', x - 5, y - 5)
        posContext.fillRect(x, y, 5, 5)
        this.$socket.emit('requestPath',{
          data: {
            start: this.start,
            end: this.end,
          }
        })
      }
    },
  },
}
</script>

