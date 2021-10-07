<template>
  <div class="patrol-history shadows">
    <div class="top-align">
      <span class="patrol-history-title">감시 히스토리</span>
      <img src="@/assets/icons/history.svg" alt="history" class="history" @click="$router.push({ name: 'History' })">
    </div>
    <div class="history-box">
      <font-awesome-icon icon="chevron-left" class="icon"/>
      <div class="history-content">
        <img src="@/assets/anormal/anormal.jpg" :onload="imageLoad()" alt="anormal image" class="history-img">
        <div class="text-content">
          <span class="date font-300">{{ date }}</span>
          <span class="date font-300">{{ time }}</span>
          <span class="anormal-text">침입자 발견</span>
        </div> 
      </div>
      <font-awesome-icon icon="chevron-right" class="icon"/>
    </div>
  </div>
</template>

<script>
export default {
  name: 'PatrolHistory',
  data() {
    return {
      date: '',
      time: '',
    }
  },
  methods: {
    imageLoad() {
      const now = new Date()
      const timezoneOffset = new Date().getTimezoneOffset() * 60 * 1000
      const todayDateTmp = new Date(now - timezoneOffset).toISOString().substring(0, 10)
      this.date = todayDateTmp
      this.time = now.getHours() + '시 ' + now.getMinutes() + '분'
    }
  }
}
</script>

<style lang="scss" scoped>
  @import './PatrolHistory.scss';
</style>