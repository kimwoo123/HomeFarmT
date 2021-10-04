<template>
  <div class="menus">
    <div class="shadows menu" @click="$router.push({ name: 'IoT' })">제어</div>
    <div class="shadows menu" @click="$router.push({ name: 'Patrol' })">감시</div>
    <div class="shadows menu" @click="$router.push({ name: 'Schedule' })">일정</div>
    <div class="shadows menu" @click="$router.push({ name: 'Control' })">조작</div>
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