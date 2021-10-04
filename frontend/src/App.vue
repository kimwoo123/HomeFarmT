<template>
  <div id="app">
    <router-view/>
  </div>
</template>

<script>
export default {
  name: 'App',
}
</script>

<style lang="scss">
  #app {
    font-family: 'Pretendard-Medium';
    color: $dark;
    width: 360px;
    height: 683px;
    margin: 0px auto;  
    background-color: white;
  }
</style>
