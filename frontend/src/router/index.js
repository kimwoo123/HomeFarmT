import Vue from 'vue'
import VueRouter from 'vue-router'
import Signup from '../views/user/Signup.vue'
import Login from '../views/user/Login.vue'
import Home from '../views/Home.vue'
import IoT from '../views/IoT.vue'
import Schedule from '../views/Schedule.vue'
import Patrol from '../views/Patrol.vue'
import NavbarLayout from '../views/NavbarLayout.vue'
import PageNotFound from '../views/PageNotFound.vue'
import Test from '../views/Test.vue'

Vue.use(VueRouter)

const routes = [
  {
    path: '*',
    redirect: '/404'
  },
  {
    path: '/404',
    name: 'PageNotFound',
    component: PageNotFound
  },
  {
    path: '/',
    redirect: Home,
  },
  {
    path: '/signup',
    name: 'Signup',
    component: Signup
  },
  {
    path: '/login',
    name: 'Login',
    component: Login
  },
  {
    path: '/home',
    name: 'Home',
    component: Home
  },
  {
    path: '/navbar-layout',
    component: NavbarLayout,
    children: [
      {
        path: '/iot',
        name: 'IoT',
        component: IoT
      },
      {
        path: '/schedule',
        name: 'Schedule',
        component: Schedule
      },
      {
        path: '/patrol',
        name: 'Patrol',
        component: Patrol
      },
    ]
  },
  {
    path: '/test',
    name: 'Test',
    component: Test
  },
]

const router = new VueRouter({
  mode: 'history',
  base: process.env.BASE_URL,
  routes
})

export default router
