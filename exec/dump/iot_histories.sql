-- MySQL dump 10.13  Distrib 8.0.25, for Win64 (x86_64)
--
-- Host: 52.79.134.74    Database: iot
-- ------------------------------------------------------
-- Server version	8.0.26

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `histories`
--

DROP TABLE IF EXISTS `histories`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `histories` (
  `historyid` int NOT NULL AUTO_INCREMENT,
  `userid` int DEFAULT NULL,
  `event_time` varchar(30) DEFAULT NULL,
  `event_title` varchar(20) DEFAULT NULL,
  `event_desc` varchar(50) DEFAULT NULL,
  `event_img` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`historyid`),
  KEY `userid` (`userid`),
  CONSTRAINT `histories_ibfk_1` FOREIGN KEY (`userid`) REFERENCES `users` (`userid`) ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=23 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `histories`
--

LOCK TABLES `histories` WRITE;
/*!40000 ALTER TABLE `histories` DISABLE KEYS */;
INSERT INTO `histories` VALUES (12,2,'2021-10-16T10:10','침입자 감지','침입자를 감지했습니다.','http://placehold.it/150x150'),(13,17,'2021-10-05T10:12','형광등 켜기','거실의 형광등을 켰습니다.',''),(14,17,'2021-10-05T10:13','침입자 감지','침입자를 감지했습니다.','http://placehold.it/150x150'),(15,17,'2021-10-05T10:23','침입자 감지','침입자를 감지했습니다.','http://placehold.it/150x150'),(16,17,'2021-10-06T02:44','에어컨 켜기','침실의 에어컨을 켰습니다.',''),(17,17,'2021-10-06T11:33','형광등 끄기','침실의 형광등을 껐습니다.',''),(18,17,'2021-10-06T12:12','침입자 감지','침입자를 감지했습니다.','http://placehold.it/150x150'),(19,17,'2021-10-06T19:20','침입자 감지','침입자를 감지했습니다.','http://placehold.it/150x150'),(20,17,'2021-10-07T18:12','티비 켜기','거실의 티비를 켰습니다.',''),(21,17,'2021-10-07T11:12','티비 끄기','거실의 티비를 껐습니다.',''),(22,17,'2021-10-07T10:12','침입자 감지','침입자를 감지했습니다.','http://placehold.it/150x150');
/*!40000 ALTER TABLE `histories` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2021-10-08  5:48:05
