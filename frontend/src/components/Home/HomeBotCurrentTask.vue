<template>
  <div class="current-task-container">
    <div class="current-task-header">Current Task</div>
    <div class="current-task-wrapper normal-shadow">
      <div class="current-task-time">
        <div>3:00</div>
        <div>|</div>
        <div>4:00</div>
      </div>
      <div class="current-task-list">
        <font-awesome-icon icon="chevron-up" />
        <div>밥 먹기</div>
        <font-awesome-icon icon="chevron-down" />
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

