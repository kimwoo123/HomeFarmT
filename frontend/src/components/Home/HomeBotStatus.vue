<template>
  <div class="bot-status-container">

    <div class="turtlebot-left-side">
      <div class="turtlebot-name-wrapper">
        <input
          v-if="!editMode"
          @keypress.enter="enterInput"
          class="turtlebot-input"
          type="text"
          v-model="turtlebotName">
        <span v-else @click="clickName">{{ turtlebotName }}</span>
      </div>
      <img 
        :src="require('../../assets/images/turtlebot.png')" 
        class="turtlebot-img">
    </div>

    <div class="turtlebot-img-wrapper">
      <HomeBotBattery/>
    </div>

  </div>
</template>

<style lang="scss" scoped>
  @import url('./HomeBotStatus.scss');
</style>

<script>
import HomeBotBattery from './HomeBotBattery.vue'
export default {
  name: 'HomeBotStatus',
  components: {
    HomeBotBattery,
  },
  data() {
    return {
      editMode: true,
      turtlebotName: 'turtlebot',
    }
  },
  methods: {
    clickName() {
      console.log('변함')
      this.editMode = false
    },
    enterInput() {
      // 여기서 터틀봇이름 변경하는 요청보내기
      this.editMode = true
    }
  }
}
</script>

