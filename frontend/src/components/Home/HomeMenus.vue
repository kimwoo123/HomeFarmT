<template>
  <div class="menus">
    <div class="shadows menu" @click="$router.push({ name: 'IoT' })">
      <img src="@/assets/icons/iot.svg" alt="iot">
      <span>제어</span>
    </div>
    <div class="shadows menu" @click="$router.push({ name: 'Patrol' })">
      <img src="@/assets/icons/patrol.svg" alt="patrol">
      <span>감시</span>  
    </div>
    <div class="shadows menu" @click="$router.push({ name: 'Schedule' })">
      <img src="@/assets/icons/schedule.svg" alt="schedule">
      <span>일정</span>  
    </div>
    <div class="shadows menu" @click="$router.push({ name: 'Control' })">
      <img src="@/assets/icons/control.svg" alt="control">
      <span>조작</span>  
    </div>
  </div>
</template>

<script>
export default {
  name: 'HomeMenus',
}
</script>

<style lang="scss" scoped>
  @import './HomeMenus.scss';
</style>