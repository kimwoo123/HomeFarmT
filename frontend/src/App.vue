<template>
  <div id="app">
    <!-- <div id="nav">
      <router-link :to="{ name: 'Login' }">로그인</router-link> |
      <router-link :to="{ name: 'Singup' }">회원가입</router-link> |
      <router-link :to="{ name: 'Home' }">홈</router-link>
    </div> -->
    <router-view/>
  </div>
</template>

<style lang="scss">
  @import url('./assets/css/normalize.scss');
  @import url('./assets/css/common.scss');
  #app {
    width: 360px;
    height: 670px;
    margin: 0px auto;
    font-family: Avenir, Helvetica, Arial, sans-serif;
    -webkit-font-smoothing: antialiased;
    -moz-osx-font-smoothing: grayscale;
    background-color: white;
  }
  body {
    background-color: grey;
  }
  ::-webkit-scrollbar { 
    display: none;
  }
</style>
