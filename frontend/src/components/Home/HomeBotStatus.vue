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
          :src="require('../../assets/turtlebot.png')" 
          class="turtlebot-img">
    </div>

    <div class="turtlebot-img-wrapper">
      
      <div>배터리 상태</div>
    </div>

  </div>
</template>
<style>
  @import url('./HomeBotStatus.scss');
</style>
<script>
export default {
  name: 'HomeBotStatus',
  components: {

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

