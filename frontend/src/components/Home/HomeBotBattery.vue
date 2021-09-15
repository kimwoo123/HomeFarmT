<template>
  <div class="battery-container">
    <span>battery</span>
    <div class="circular">
      <div class="inner">{{ batteryPercentage }}%</div>
      <div class="circle">
        <div class="bar left">
          <div class="progress" :style="leftRotate"></div>
        </div>
        <div class="bar right">
          <div class="progress" :style="rightRotate"></div>
        </div>
      </div>
    </div>
  </div>
</template>
<style lang="scss" scoped>
  @import './HomeBotBattery.scss';
</style>
<script>
export default {
  name: 'HomeBotBattery',
  data() {
    return {
      batteryPercentage: 90,
      degreeLeft: 0,
      degreeRight: 0,
    }
  },
  computed: {
    leftRotate()  {
      // ${this.degreeLeft}
      return { transform : `rotate(${this.degreeLeft}deg)`}
    },
    rightRotate() {
      // ${this.degreeRight}
      return { transform: `rotate(${this.degreeRight}deg)`}
    }
  },
  mounted() {
    if (this.batteryPercentage <= 50) {
      this.degreeLeft = (180 / 50) * this.batteryPercentage
      this.degreeRight = 0
    } else if (this.batteryPercentage >= 50) {
      this.degreeLeft = 180
      this.degreeRight = (180 / 50) * (this.batteryPercentage - 50)
    }
    console.log(this.degreeLeft, this.degreeRight)
  },
}
</script>

