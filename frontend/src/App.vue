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
  @import '@/assets/css/normalize.scss';
  @import '@/assets/css/common.scss';
  #app {
    width: 360px;
    height: 683px;
    margin: 0px auto;
    padding: 8px;
    font-family: Avenir, Helvetica, Arial, sans-serif;
    -webkit-font-smoothing: antialiased;
    -moz-osx-font-smoothing: grayscale;
    background-color: white;
  }
  body {
    background-color: grey;
  }
  #nav {
    padding: 15px;
  }

  #nav a {
    font-weight: bold;
    color: #2c3e50;
  }

  .normal-shadow {
    box-shadow: rgba(17, 17, 26, 0.05) 0px 1px 0px, rgba(17, 17, 26, 0.1) 0px 0px 8px;
  }

  #nav a.router-link-exact-active {
    color: #42b983;
  }

  ::-webkit-scrollbar { 
    display: none;
  }
</style>
