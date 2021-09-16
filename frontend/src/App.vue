<template>
  <div id="app">
    <router-view/>
  </div>
</template>

<script>
// import { useLazyQuery } from '@vue/apollo-composable'

export default {
  name: 'App',
}
</script>
<style lang="scss">
  @import '@/assets/scss/normalize.scss';
  @import '@/assets/scss/common.scss';
  
  #app {
    width: 360px;
    height: 683px;
    margin: 0px auto;
    font-family: Avenir, Helvetica, Arial, sans-serif;
    -webkit-font-smoothing: antialiased;
    -moz-osx-font-smoothing: grayscale;
    background-color: white;
  }
  body {
    background-color: grey;
  }
  .normal-shadow {
    box-shadow: rgba(17, 17, 26, 0.05) 0px 1px 0px, rgba(17, 17, 26, 0.1) 0px 0px 8px;
  }

  ::-webkit-scrollbar { 
    display: none;
  }
</style>
