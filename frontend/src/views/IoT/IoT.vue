<template>
  <div>
    <Navbar :title="'기기 제어'" :left_icon="true" :type="'control'" :right_text="'추가'" :left_push="'Home'" :right_push="'IoTNew'"/>
    
    <div class="position-container">
      <span v-for="(position, idx) in positions" :key="idx" @click="changePosition(position)"
        class="position" :class="{ 'position-active' : position === currentPosition }">{{ position }}</span>
    </div>
    
    <!-- props로 currentPositions 넘겨주기 -->
    <div class="iot-container">
      <IoTButton/>
    </div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import IoTButton from '@/components/IoT/IoTButton.vue'

export default {
  name: 'IoT',
  components: {
    IoTButton,
    Navbar,
  },
  data() {
    return {
      positions: [
        '전체',
        '부엌',
        '거실',
        '침실',
      ],
      currentPosition: '전체'
    }
  },
  methods: {
    changePosition(position) {
      this.currentPosition = position
    }
  }
}
</script>

<style lang="scss" scoped>
  @import "./IoT.scss";
</style>