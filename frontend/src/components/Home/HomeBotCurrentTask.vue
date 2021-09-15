<template>
  <div class="current-task-container">
    <div class="current-task-header">Next Task</div>
    <div class="current-task-wrapper normal-shadow">
      <div class="current-task-time">
        <div>3:00</div>
        <div>|</div>
        <div>4:00</div>
      </div>
      <div class="current-task-list">
        <button class="task-slide-button">위</button>
        <div>밥 먹기</div>
        <button class="task-slide-button">아래</button>
      </div>
    </div>
  </div>
</template>
<style lang="scss" scoped>
  @import './HomeBotCurrentTask.scss';
</style>
<script>
export default {
  name: 'HomeBotCurrentTask',
}
</script>

