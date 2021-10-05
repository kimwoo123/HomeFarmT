const { History } = require("../../models/index")
const { User } = require("../../models/index")

module.exports = {
  Query: {
    getHistory: async (_, args, context) => {
      if (!context) {
        throw new Error('로그인이 필요합니다')
      }
      const user = await User.findOne({ where: { email: context.hashedEmail }})
      const userid = user.dataValues.userid
      const userHistory = await History.findAll({ where: { userid: userid }});
      let historyList = []
      for (let history of userHistory ) {
        historyList.push(history.dataValues)
      }
      return historyList
    }
  },
  Mutation: {
    createHistory: async (_, { event_time, event_title, event_desc, event_img }, context) => {
      if (!context) {
        throw new Error('인증된 유저가 아닙니다')
      }
      const user = await User.findOne({ where: { email: context.hashedEmail }})
      const userid = user.dataValues.userid
      const newHistory = await History.create({ 
        event_time: event_time,
        event_title: event_title,
        event_desc: event_desc,
        event_img: event_img,
        userid: userid
       })
      return newHistory
    },
    deleteSchedule: async (_, { historyid }) => {
      await Schedule.destroy({ where: { historyid }})
    }
  }
};
