<template>
  <div class="menus-container">
    <div class="menu-name">Actions</div>
    <div class="menus-wrapper">
      <div class="menu one" @click="clickMenuButton" data-menu-name="Patrol">Patrol</div>
      <div class="menu two" @click="clickMenuButton" data-menu-name="Schedule">Schedule</div>
      <div class="menu two" @click="clickMenuButton" data-menu-name="IoT">IoT</div>
      <div class="menu one">메뉴 4</div>
    </div>
  </div>
</template>
<style scoped lang="scss">
  @import './HomeMenus.scss';
</style>
<script>
export default {
  name: 'HomeMenus',
  methods: {
    clickMenuButton (event) {
      const menu = event.target.dataset.menuName
      this.$router.push({name: menu})
    },
  }
}
</script>

