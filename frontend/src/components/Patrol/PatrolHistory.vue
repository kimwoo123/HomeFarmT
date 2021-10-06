<template>
  <div class="patrol-history shadows">
    <div class="top-align">
      <span class="patrol-history-title">감시 히스토리</span>
      <img src="@/assets/icons/history.svg" alt="history" class="history" @click="$router.push({ name: History })">
    </div>
    <div class="history-box">
      <font-awesome-icon icon="chevron-left" class="icon"/>
      <div class="history-content">
        <img src="@/assets/images/patrol.png" alt="anormal image" class="history-img">
        <div class="text-content">
          <span class="date font-300">2021년 3월 7일</span>
          <span class="date font-300">11시 20분</span>
          <span class="anormal-text">침입자 발견</span>
        </div> 
      </div>
      <font-awesome-icon icon="chevron-right" class="icon"/>
    </div>
  </div>
</template>

<script>
export default {

}
</script>

<style lang="scss" scoped>
  @import './PatrolHistory.scss';
</style>